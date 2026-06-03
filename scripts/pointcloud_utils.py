#!/usr/bin/env python3
"""
Point cloud utilities for segmented object grasping.

Builds a world-frame point cloud from a SAM3 mask + RealSense depth image,
then computes centroid, principal axes, and gripper orientation via PCA.

References magpie_perception.pcd (correlllab/magpie_perception) for the full
6D PCA pose frame — use get_full_pose() when angled grasps are needed.
"""

import numpy as np


def build_segmented_pcd(mask, depth_mm, caminfo_k, tcp_matrix, tcp_to_cam):
    """
    Build a world-frame point cloud from a SAM3 mask using open3d's RGBD pipeline.

    Args:
        mask:        (H, W) bool   — SAM3 segmentation mask
        depth_mm:    (H, W) uint16 — RealSense depth in millimetres
        caminfo_k:   9-element flat camera intrinsics (CameraInfo.k)
        tcp_matrix:  (4, 4) TCP pose in world frame
        tcp_to_cam:  (4, 4) fixed TCP→camera extrinsic

    Returns:
        points: (N, 3) float64 XYZ array in world frame
        pcd:    open3d.geometry.PointCloud
    """
    import open3d as o3d

    fx, fy = caminfo_k[0], caminfo_k[4]
    cx, cy = caminfo_k[2], caminfo_k[5]

    ys, xs = np.where(mask)
    zs = depth_mm[ys, xs].astype(np.float64) / 1000.0  # mm → m

    valid = zs > 0
    xs, ys, zs = xs[valid], ys[valid], zs[valid]

    if len(zs) == 0:
        pcd = o3d.geometry.PointCloud()
        return np.zeros((0, 3)), pcd

    # Back-project each masked pixel to camera frame
    x_cam = (xs - cx) * zs / fx
    y_cam = (ys - cy) * zs / fy
    pts_cam = np.stack([x_cam, y_cam, zs, np.ones_like(zs)], axis=1)  # (N, 4)

    # Transform to world frame
    T = tcp_matrix @ tcp_to_cam
    pts_world = (T @ pts_cam.T).T[:, :3]  # (N, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_world)

    return pts_world, pcd


def denoise_pcd(pcd, nb_neighbors=30, std_ratio=0.5):
    """Remove statistical outliers then keep only the largest DBSCAN cluster.
    Returns cleaned PointCloud and numpy points."""
    import open3d as o3d

    cleaned, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio
    )

    # Keep only the largest connected cluster — removes stray gripper/cable points
    # that survive statistical filtering but form their own dense cluster.
    labels = np.array(cleaned.cluster_dbscan(eps=0.012, min_points=5))
    if labels.max() >= 0:
        counts = np.bincount(labels[labels >= 0])
        largest = int(np.argmax(counts))
        cleaned = cleaned.select_by_index(np.where(labels == largest)[0])

    return cleaned, np.asarray(cleaned.points)


def analyse_pcd(pcd_or_points):
    """
    Compute centroid and principal axes (PCA) for grasp orientation.

    Follows magpie_perception.pcd.get_segment:
      - Uses open3d's compute_mean_and_covariance() for accuracy
      - Applies the -π/2 XY coordinate swap on centroid:
          grasp_pos = [Y, -X, Z]  (accounts for camera mount rotation)
      - get_full_pose() for the full 6D frame (angled grasps)

    Args:
        pcd_or_points: open3d PointCloud OR (N, 3) numpy array

    Returns dict:
        centroid        (3,)   grasp centroid in robot frame (XY-swapped)
        axes            (3, 3) eigenvectors sorted by variance descending
        eigenvalues     (3,)   variance along each axis
        extent_m        (3,)   object extent in metres [major, minor, normal]
        grasp_angle_deg float  wrist Z rotation (fingers perpendicular to major axis)
    """
    import open3d as o3d

    if isinstance(pcd_or_points, o3d.geometry.PointCloud):
        pcd = pcd_or_points
        points = np.asarray(pcd.points)
    else:
        points = np.asarray(pcd_or_points, dtype=np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

    if len(points) < 3:
        raise ValueError(f'Need at least 3 points for PCA, got {len(points)}')

    # Use open3d built-in — more accurate than np.cov on large point clouds
    mean, cov = pcd.compute_mean_and_covariance()

    # Points are already in world frame (transformed in build_segmented_pcd).
    # No coordinate swap needed — that only applies in mentor's wrist-frame pipeline.
    centroid = np.asarray(mean)

    eigenvalues, eigenvectors = np.linalg.eigh(cov)  # eigh: symmetric → real eigenvalues

    # Sort descending — largest variance (longest axis) first
    idx = np.argsort(eigenvalues)[::-1]
    eigenvalues  = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]

    major_axis = eigenvectors[:, 0]

    # Gripper rotation: fingers close perpendicular to major axis in XY plane
    angle_rad = np.arctan2(major_axis[1], major_axis[0]) + np.pi / 2
    grasp_angle_deg = float(np.degrees(angle_rad) % 180)

    centered = points - mean  # use raw mean for extent, not swapped centroid
    projections = centered @ eigenvectors
    extent_m = projections.max(axis=0) - projections.min(axis=0)

    return dict(
        centroid=centroid,
        axes=eigenvectors,
        eigenvalues=eigenvalues,
        extent_m=extent_m,
        grasp_angle_deg=grasp_angle_deg,
    )


def get_full_pose(pcd_or_points):
    """
    Full 6D PCA pose frame following magpie_perception.pcd.get_segment.
    Handles axis alignment to world frame and right-hand rule enforcement.
    Use this for angled grasps where all three axes matter.

    Returns 4×4 transform with PCA axes as rotation + grasp centroid.
    """
    import io
    import contextlib
    import open3d as o3d
    from magpie_perception.pcd import get_pca_frame

    if isinstance(pcd_or_points, o3d.geometry.PointCloud):
        pcd = pcd_or_points
    else:
        points = np.asarray(pcd_or_points, dtype=np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

    mean, cov = pcd.compute_mean_and_covariance()

    # Suppress debug prints inside get_pca_frame
    with contextlib.redirect_stdout(io.StringIO()):
        _, tmat = get_pca_frame(np.array(mean), cov, scale=1.0)

    # Apply the same -π/2 XY centroid swap as get_segment
    tmat[:3, 3] = [mean[1], -mean[0], mean[2]]

    return tmat


def smart_grasp_angle(pca_result, object_name='', image_rgb=None,
                      gemini_client=None, gemini_model='gemini-2.5-flash'):
    """
    Compute the best grasp angle using shape geometry + optional Gemini classification.

    Three strategies:
      symmetric  — cube/ball/cylinder: any angle works → 0°
      short_side — rectangle/book: grip perpendicular to long axis → PCA angle (default)
      long_side  — pen/banana: grip parallel to long axis → PCA angle + 90°

    Steps:
      1. Geometry rule: if major/minor < 1.3 → symmetric
      2. Gemini override: if client + image provided, classify shape for complex objects

    Returns (angle_deg, strategy, reason).
    """
    import cv2, base64, tempfile, os

    major = pca_result['extent_m'][0]
    minor = pca_result['extent_m'][1]
    ratio = major / max(minor, 1e-6)
    pca_angle = pca_result['grasp_angle_deg']

    # ── 1. Geometry default ───────────────────────────────────────────────────
    if ratio < 1.3:
        strategy = 'symmetric'
        reason   = f'geometry: ratio={ratio:.2f} < 1.3 (cube/cylinder)'
    else:
        strategy = 'short_side'
        reason   = f'geometry: ratio={ratio:.2f} (rectangle)'

    # ── 2. Gemini override ────────────────────────────────────────────────────
    if gemini_client is not None and image_rgb is not None and object_name:
        try:
            from google.genai import types as gtypes
            tmp = tempfile.mktemp(suffix='.jpg')
            cv2.imwrite(tmp, cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
            with open(tmp, 'rb') as fh:
                img_bytes = fh.read()
            os.unlink(tmp)

            prompt = (
                f'Object: "{object_name}"\n'
                f'Point cloud extents: major={major*1000:.0f}mm, minor={minor*1000:.0f}mm '
                f'(ratio={ratio:.2f})\n\n'
                f'Choose the best gripper strategy — reply with EXACTLY one word:\n'
                f'  symmetric  — shape is round/square, any angle works (cube, ball, cylinder, bottle cap)\n'
                f'  short_side — grip perpendicular to longest dimension (box, book, phone, brick)\n'
                f'  long_side  — grip parallel to longest dimension (pen, banana, screwdriver, remote)\n'
            )
            r = gemini_client.models.generate_content(
                model=gemini_model,
                contents=[
                    gtypes.Part.from_bytes(data=img_bytes, mime_type='image/jpeg'),
                    prompt,
                ])
            word = r.text.strip().lower().split()[0]
            if word in ('symmetric', 'short_side', 'long_side'):
                reason = f'gemini: {word} (ratio={ratio:.2f})'
                strategy = word
        except Exception as e:
            reason += f' [gemini failed: {e}]'

    # ── 3. Apply strategy ─────────────────────────────────────────────────────
    # pca_angle already has +90° built in (analyse_pcd adds π/2 to major axis angle).
    # Empirically: pca_angle grips the SHORT faces; pca_angle+90° grips the LONG faces.
    if strategy in ('symmetric', 'short_side'):
        angle = (pca_angle + 90.) % 180.
    else:  # long_side
        angle = pca_angle

    angle = float(angle) % 90.   # wrist limit: 0-90° covers all orientations

    # Symmetric shapes have no preferred PCA direction — snap to 0° for consistency
    if strategy == 'symmetric':
        angle = 0.0

    return angle, strategy, reason


def grasp_rotation_matrix(grasp_angle_deg):
    """
    3×3 rotation matrix for straight-down gripper approach with wrist rotated
    by grasp_angle_deg around world Z so fingers grip perpendicular to major axis.

    Derivation: start from base straight-down orientation (tool Z = world -Z,
    tool X = world +X), apply rotation by θ around world Z.

        R = [[cos θ,  sin θ,  0],
             [sin θ, -cos θ,  0],
             [0,      0,     -1]]

    θ=0 → fingers along world X, θ=90 → fingers along world Y.
    """
    theta = np.radians(grasp_angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c,  s, 0],
        [ s, -c, 0],
        [ 0,  0, -1],
    ])


def print_pcd_results(result, n_points):
    """Pretty-print analyse_pcd output."""
    c = result['centroid']
    e = result['extent_m']
    axes = result['axes']

    print(f'\n=== POINT CLOUD ANALYSIS ===')
    print(f'  Points (after denoise): {n_points}')
    print(f'  Centroid (world):  x={c[0]:.3f}  y={c[1]:.3f}  z={c[2]:.3f} m')
    print(f'  Extent:   major={e[0]*100:.1f} cm  minor={e[1]*100:.1f} cm  depth={e[2]*100:.1f} cm')
    print(f'  Major axis direction: [{axes[0,0]:.2f}, {axes[1,0]:.2f}, {axes[2,0]:.2f}]')
    print(f'  Grasp angle (wrist Z): {result["grasp_angle_deg"]:.1f} deg')
    print(f'============================')
