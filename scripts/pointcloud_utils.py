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


def denoise_pcd(pcd, nb_neighbors=30, std_ratio=0.5, hint=None):
    """Remove statistical outliers. Returns cleaned PointCloud and numpy points."""
    cleaned, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio
    )
    return cleaned, np.asarray(cleaned.points)


def check_view_quality(pts_cleaned, mask, depth_mm, min_points=80, min_fill=0.25):
    """
    Check whether the camera has a clean view of the object.

    Detects two obstruction modes:
      1. Too few valid depth pixels inside the SAM3 mask  (edge/cable blocking)
      2. Too few points surviving denoise  (sparse / noisy scan)

    Args:
        pts_cleaned: (N, 3) array after denoise_pcd
        mask:        (H, W) bool SAM3 mask
        depth_mm:    (H, W) uint16 RealSense depth
        min_points:  minimum denoised points to be considered usable
        min_fill:    minimum fraction of mask pixels with valid depth

    Returns dict:
        ok      bool   — True if view is good
        reason  str    — human-readable diagnosis
        n_pts   int    — denoised point count
        fill    float  — fraction of mask pixels with valid depth
    """
    mask_px = int(mask.sum())
    if mask_px == 0:
        return dict(ok=False, reason='mask is empty', n_pts=0, fill=0.0)

    valid_depth = (depth_mm[mask] > 0).sum()
    fill = float(valid_depth) / mask_px
    n_pts = len(pts_cleaned)

    if fill < min_fill:
        return dict(ok=False, reason=f'depth fill {fill:.0%} < {min_fill:.0%} — view blocked',
                    n_pts=n_pts, fill=fill)
    if n_pts < min_points:
        return dict(ok=False, reason=f'only {n_pts} pts after denoise — view blocked or object tiny',
                    n_pts=n_pts, fill=fill)

    return dict(ok=True, reason='ok', n_pts=n_pts, fill=fill)


def top_layer(pts, percentile=50):
    """Return only points in the top Z percentile — strips mat/table layer.
    Use this before analyse_pcd for angle computation only; keep full pts for position."""
    if len(pts) == 0:
        return pts
    z = pts[:, 2]
    if z.max() - z.min() < 0.020:
        return pts   # single layer, no filtering needed
    return pts[z >= np.percentile(z, percentile)]


def mask_grasp_angle(mask):
    """
    Compute grasp angle from the 2D SAM3 mask shape (image-space PCA).

    This is more robust than point-cloud PCA when the depth camera produces
    stripe artifacts (e.g. RealSense IR structured light on flat surfaces).
    Returns angle_deg in the same convention as analyse_pcd (0–180°).

    Args:
        mask: (H, W) bool — SAM3 segmentation mask

    Returns:
        angle_deg: float — PCA major axis angle (0–180°)
        ratio:     float — major/minor extent ratio
    """
    ys, xs = np.where(mask)
    if len(xs) < 10:
        return 0.0, 1.0
    pts2d = np.stack([xs, ys], axis=1).astype(np.float64)
    mean2d = pts2d.mean(axis=0)
    cov2d  = np.cov((pts2d - mean2d).T)
    eigvals, eigvecs = np.linalg.eigh(cov2d)
    idx = np.argsort(eigvals)[::-1]
    eigvals, eigvecs = eigvals[idx], eigvecs[:, idx]
    major_vec = eigvecs[:, 0]   # (dx_img, dy_img)
    # Image x → world X, image y → world Y (approximate for top-down view)
    # Negate dy because image Y increases downward, world Y increases away from robot
    angle_rad = np.arctan2(-major_vec[1], major_vec[0]) + np.pi / 2
    angle_deg = float(np.degrees(angle_rad) % 180)
    ratio = float(np.sqrt(eigvals[0] / max(eigvals[1], 1e-9)))
    return angle_deg, ratio


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
                      gemini_client=None, gemini_model='gemini-2.5-flash',
                      mask=None):
    """
    Compute the best grasp angle using shape geometry + optional Gemini classification.

    Three strategies:
      symmetric  — cube/ball: use mask PCA angle (grips nearest flat face)
      short_side — rectangle/book: grip perpendicular to long axis
      long_side  — pen/banana: grip parallel to long axis

    Steps:
      1. Geometry rule: if major/minor < 1.3 → symmetric
      2. Gemini override: classify shape for complex objects
      3. Angle source: mask 2D PCA (stripe-free) if mask provided, else point cloud PCA

    Returns (angle_deg, strategy, reason).
    """
    import cv2, base64, tempfile, os

    major = pca_result['extent_m'][0]
    minor = pca_result['extent_m'][1]
    ratio = major / max(minor, 1e-6)
    pca_angle = pca_result['grasp_angle_deg']

    # Use mask-based 2D PCA angle when available — avoids IR stripe artifacts
    if mask is not None:
        mask_angle, mask_ratio = mask_grasp_angle(mask)
        pca_angle = mask_angle
        if ratio < 1.3:
            ratio = mask_ratio   # also update ratio from cleaner mask data

    # ── 1. Geometry default ───────────────────────────────────────────────────
    # If extents are too small the point cloud is noise (e.g. IR-opaque object
    # on a textured mat) — can't trust PCA angle, force symmetric.
    if major < 0.025:
        strategy = 'symmetric'
        reason   = f'geometry: extent too small ({major*1000:.0f}mm) — point cloud unreliable, forcing symmetric'
    elif ratio < 1.3:
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
    if strategy == 'symmetric':
        angle = pca_angle   # use PCA even for cubes — grips nearest flat face
    elif strategy == 'short_side':
        angle = (pca_angle + 90.) % 180.   # grip across short dim → contact long faces
    else:  # long_side
        angle = pca_angle                   # grip across long dim → contact short faces

    angle = float(angle) % 90.   # wrist limit: symmetric jaws so 0-90° covers all orientations
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
