"""
Shared full-run grasp executor for the method comparison (notebooks/grasp_compare.ipynb).

Every method (PCA baseline / Contact-GraspNet / AnyGrasp / GraspGen) is run through the
SAME pipeline — capture one scene, plan a grasp with the given detector, execute the
same approach→descend→close→lift, score the outcome — so the only variable is the
grasp PLANNER. That makes "which method should be main?" a fair comparison.

Usage (in the magpie kernel, after c01–c04):
    from grasp_runner import capture_scene, full_run
    scene = capture_scene(node, sam3_query, ACTIVE)        # once; reuse for all methods
    res = full_run(node, detector, scene, CONST)           # run + grasp + outcome
"""

import time

import numpy as np


def capture_scene(node, sam3_query, object_name, tcp_to_cam=None):
    """Snapshot the current top-down view → world point cloud + metadata, shared by all
    detectors so they plan from identical input."""
    from pointcloud_utils import build_segmented_pcd, denoise_pcd
    node.spin(6)
    color = node.color.copy(); depth = node.depth.copy()
    tcp = node.tcp.copy(); ci = node.caminfo
    boxes, scores, mask = sam3_query(color, object_name)
    if mask is None or not mask.any():
        raise RuntimeError(f'no detection for "{object_name}"')
    _, raw = build_segmented_pcd(mask, depth, ci.k, tcp, tcp_to_cam)
    _, pts = denoise_pcd(raw)
    return dict(points=np.asarray(pts), mask=mask, color=color, depth=depth,
                tcp=tcp, k=ci.k, object=object_name)


def _grasp_pose_from(grasp, scene, const):
    """Turn a detector Grasp into an executable world TCP target with a sane grasp depth.
    Learned detectors give full 6-DOF; the PCA baseline gives yaw+centroid, so we set the
    descend Z from the cloud + table here."""
    pose = np.array(grasp.pose, dtype=float).copy()
    pts = scene['points']
    obj_top = float(np.percentile(pts[:, 2], 90))
    table_z = const.get('TABLE_Z') or float(np.percentile(pts[:, 2], 5))
    obj_h = max(obj_top - table_z, 0.010)
    grasp_off = float(np.clip(obj_h / 2., 0.010, 0.040))
    # PCA pose Z is the centroid/top; lower it to the grasp depth. Learned poses already
    # encode depth, but clamping to a safe floor is still good hygiene.
    gfz = max(obj_top - grasp_off, const['HARD_FLOOR_Z'])
    pose[2, 3] = gfz + const['GRIPPER_LEN']
    return pose


def execute_grasp(node, grasp_pose, width, force, const, rclpy):
    """Open → approach above → descend to grasp → close at `force` → lift. Returns outcome."""
    import numpy as np
    t0 = time.time()
    ap = grasp_pose.copy(); ap[2, 3] += const['APPROACH_H']
    node.open_g(); time.sleep(0.4)
    node.move(ap, spd=0.08); time.sleep(0.3); node.spin(3)
    node.move(grasp_pose, spd=0.05); time.sleep(0.4); node.spin(3)
    # close
    node.clear_err(); node.set_force(float(force)); node.close_g(); time.sleep(2.0)
    for _ in range(6):
        rclpy.spin_once(node, timeout_sec=0.15)
    s = node.gs
    contact_force = float(getattr(s, 'force', 0.) or 0.)
    # lift
    node.set_force(float(force) + 0.5)
    node.move(ap, spd=0.05); time.sleep(0.4); node.spin(6)
    s2 = node.gs
    ap_mm = float(getattr(s2, 'position', 0.) or 0.)
    held = (getattr(s2, 'force', 0.) or 0.) >= 0.3 and ap_mm >= 3.0
    # gentle place-back at the SAME spot: lower to grasp height, release, then rise —
    # don't drop the object from the lifted height.
    node.move(grasp_pose, spd=0.04); time.sleep(0.4)
    node.open_g(); time.sleep(0.6)
    node.move(ap, spd=0.05); time.sleep(0.2)
    return dict(held=bool(held), grasp_force=contact_force, final_aperture=ap_mm,
                exec_time_s=round(time.time() - t0, 2))


def full_run(node, detector, scene, const, rclpy, force=4.0, place_back=True):
    """Plan with `detector` on the shared scene, execute, return a comparable result row."""
    res = detector.detect(scene['points'], colors=None, mask=scene['mask'],
                          tcp=scene['tcp'], intrinsics=scene['k'])
    row = dict(detector=detector.name, n_grasps=res.n,
               plan_latency_ms=round(1000 * res.latency_s, 1))
    if not res.best:
        row.update(held=None, note='no grasp produced')
        return row
    g = res.best
    pose = _grasp_pose_from(g, scene, const)
    row.update(score=round(g.score, 3), angle_deg=round(g.angle_deg(), 1),
               width_mm=round(g.width * 1000, 1))
    out = execute_grasp(node, pose, g.width * 1000 + 8, force, const, rclpy)
    row.update(out)
    return row    # execute_grasp already placed the object back gently at the grasp spot
