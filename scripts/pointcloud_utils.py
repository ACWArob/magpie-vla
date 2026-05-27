#!/usr/bin/env python3
"""
Point cloud utilities for segmented object grasping.

Given a SAM3 binary mask + RealSense depth image + camera intrinsics + TCP pose,
back-projects masked pixels into a 3D world-frame point cloud, then computes:
  - Centroid (XYZ position of object centre)
  - Principal axes via PCA (object orientation)
  - Recommended gripper rotation around world Z (perpendicular to major axis)
"""

import numpy as np


def build_segmented_pcd(mask, depth_mm, caminfo_k, tcp_matrix, tcp_to_cam):
    """
    Back-project SAM3 mask pixels into world-frame 3D points.

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

    # Back-project to camera frame
    x_cam = (xs - cx) * zs / fx
    y_cam = (ys - cy) * zs / fy

    pts_cam = np.stack([x_cam, y_cam, zs, np.ones_like(zs)], axis=1)  # (N, 4)

    T = tcp_matrix @ tcp_to_cam
    pts_world = (T @ pts_cam.T).T[:, :3]  # (N, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_world)

    return pts_world, pcd


def denoise_pcd(pcd, nb_neighbors=20, std_ratio=2.0):
    """
    Remove statistical outliers (stray depth noise points).
    Returns cleaned PointCloud and numpy points.
    """
    cleaned, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio
    )
    return cleaned, np.asarray(cleaned.points)


def analyse_pcd(points):
    """
    Compute centroid and principal axes (PCA) of a point cloud.

    Args:
        points: (N, 3) float array in world frame

    Returns dict:
        centroid        (3,)   XYZ of object centre in world frame
        axes            (3, 3) eigenvectors as columns, sorted by variance descending
                               axes[:, 0] = major axis (longest dimension)
                               axes[:, 1] = minor axis
                               axes[:, 2] = normal axis (into surface)
        eigenvalues     (3,)   variance along each axis
        extent_m        (3,)   object extent in metres along each axis [major, minor, normal]
        grasp_angle_deg float  gripper rotation around world Z so fingers grip
                               perpendicular to the major axis (0–180°)
    """
    points = np.asarray(points, dtype=np.float64)
    if len(points) < 3:
        raise ValueError(f'Need at least 3 points for PCA, got {len(points)}')

    centroid = points.mean(axis=0)
    centered = points - centroid

    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    # Sort descending — largest variance (longest axis) first
    idx = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]

    major_axis = eigenvectors[:, 0]

    # Gripper rotation: fingers close perpendicular to major axis
    # Project major axis onto XY plane → add 90° → that's the finger closure direction
    angle_rad = np.arctan2(major_axis[1], major_axis[0]) + np.pi / 2
    grasp_angle_deg = float(np.degrees(angle_rad) % 180)

    # Extent: range of projections onto each principal axis
    projections = centered @ eigenvectors  # (N, 3)
    extent_m = projections.max(axis=0) - projections.min(axis=0)

    return dict(
        centroid=centroid,
        axes=eigenvectors,
        eigenvalues=eigenvalues,
        extent_m=extent_m,
        grasp_angle_deg=grasp_angle_deg,
    )


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
