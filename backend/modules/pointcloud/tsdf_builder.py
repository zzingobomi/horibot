"""ICP + multi-way PoseGraph optimization + TSDF mesh build.

흐름:
  1. scan별 (rgbd, intrinsic, T_base_cam_init) 준비
     - raw_motor_positions → JointCoordinates.motor_to_urdf → arm rad
     - PybulletSolver.fk_to_matrix(arm_rad) → (R, t) — sag+link 적용된 actual ee
     - T_base_cam_init = T_base_ee · T_ee_cam (hand_eye)
  2. depth bilateral filter (edge 보존 + stereo 노이즈 ↓)
  3. RGBD → PointCloud (cam frame) + normal 추정 (point-to-plane ICP 필수)
  4. pair-wise point-to-plane ICP — 인접 페어 + 거리 임계 안의 loop closure 후보
  5. PoseGraph 빌드 → global_optimization (Levenberg-Marquardt)
  6. refined T_base_cam으로 ScalableTSDFVolume.integrate
  7. extract_triangle_mesh + cluster_connected_triangles로 작은 fragment 정리
  8. PLY 저장

Open3D 좌표계:
  - TSDF.integrate(rgbd, intrinsic, extrinsic) → extrinsic = T_cam←world
  - ICP transformation = T_target←source
  - PoseGraph node.pose = T_world←cam_i = T_base←cam_i
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from core.joint_coordinates import JointCoordinates
from modules.calibration.loader import load_calibration
from modules.motor.motor_config import MotorConfig
from modules.kinematics.solver import PybulletSolver

logger = logging.getLogger(__name__)

# ─── 디폴트 (PointCloud guide § 9) ───
DEFAULT_VOXEL_SIZE = 0.002  # 2mm
DEFAULT_SDF_TRUNC = 0.010  # 10mm (= 5 × voxel)
DEFAULT_DEPTH_TRUNC = 0.5  # m — D405 sweet spot 안
DEFAULT_ICP_MAX_DIST = 0.010  # m — voxel 5배
DEFAULT_BILATERAL_DIAMETER = 5
DEFAULT_BILATERAL_SIGMA_COLOR = 50.0
DEFAULT_BILATERAL_SIGMA_SPACE = 50.0
PAIR_UNCERTAIN_DIST = 0.15  # m — 비인접 페어 ICP 후보 거리 임계
ICP_FITNESS_FLOOR = 0.3  # loop closure edge 기각 임계
MIN_TRIANGLE_CLUSTER_SIZE = 500
MIN_SCANS = 2  # PoseGraph optimization 의미 있으려면 최소 2자세


@dataclass
class BuildResult:
    path: Path
    vertex_count: int
    triangle_count: int
    n_scans: int
    n_edges: int
    elapsed: float = 0.0
    refined_poses: list[np.ndarray] = field(default_factory=list)


def build_mesh(
    scans: list[dict],
    arm_cfgs: list[MotorConfig],
    out_path: Path,
    *,
    voxel_size: float = DEFAULT_VOXEL_SIZE,
    sdf_trunc: float = DEFAULT_SDF_TRUNC,
    depth_trunc: float = DEFAULT_DEPTH_TRUNC,
    icp_max_dist: float = DEFAULT_ICP_MAX_DIST,
    bilateral_diameter: int = DEFAULT_BILATERAL_DIAMETER,
) -> BuildResult:
    if len(scans) < MIN_SCANS:
        raise ValueError(
            f"scan {MIN_SCANS}개 이상 필요. 현재: {len(scans)}"
        )

    calib = load_calibration()
    if calib.hand_eye is None:
        raise RuntimeError("hand_eye.npz 없음 — 캘리브 먼저 진행")

    T_ee_cam = np.eye(4)
    T_ee_cam[:3, :3] = calib.hand_eye.R
    T_ee_cam[:3, 3] = calib.hand_eye.t.reshape(3)

    solver = PybulletSolver()
    coords = JointCoordinates()

    cfg_by_id = {cfg.id: cfg for cfg in arm_cfgs}

    T_base_cam_init: list[np.ndarray] = []
    rgbds: list[o3d.geometry.RGBDImage] = []
    intrinsics: list[o3d.camera.PinholeCameraIntrinsic] = []
    pcds_down: list[o3d.geometry.PointCloud] = []

    # ─── 1~3. 자세별 init pose + RGBD + cloud + normal ──────────────
    for idx, s in enumerate(scans):
        raw_positions = s["raw_motor_positions"]
        scan_arm_ids = s["arm_motor_ids"]
        arm_rad: list[float] = []
        for raw, mid in zip(raw_positions, scan_arm_ids):
            cfg = cfg_by_id.get(int(mid))
            if cfg is None:
                raise RuntimeError(
                    f"scan #{idx} motor id {mid}가 현재 arm_cfgs에 없음"
                )
            arm_rad.append(coords.motor_to_urdf(int(raw), cfg))

        R_be, t_be = solver.fk_to_matrix(arm_rad)
        T_base_ee = np.eye(4)
        T_base_ee[:3, :3] = np.asarray(R_be)
        T_base_ee[:3, 3] = np.asarray(t_be)
        T_bc = T_base_ee @ T_ee_cam
        T_base_cam_init.append(T_bc)

        # depth bilateral (edge 보존)
        depth_f = s["depth_z16"].astype(np.float32)
        depth_filtered = cv2.bilateralFilter(
            depth_f,
            bilateral_diameter,
            DEFAULT_BILATERAL_SIGMA_COLOR,
            DEFAULT_BILATERAL_SIGMA_SPACE,
        ).astype(np.uint16)

        color_rgb = np.ascontiguousarray(s["color_bgr"][:, :, ::-1])
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_rgb),
            o3d.geometry.Image(depth_filtered),
            depth_scale=1.0 / s["depth_scale"],
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False,
        )
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            s["width"], s["height"], s["fx"], s["fy"], s["cx"], s["cy"]
        )
        rgbds.append(rgbd)
        intrinsics.append(intrinsic)

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
        pcd_down = pcd.voxel_down_sample(voxel_size)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 2.0, max_nn=30
            )
        )
        pcds_down.append(pcd_down)

    n = len(scans)

    # ─── 4. Pair-wise ICP (인접 + 가까운 loop closure 후보) ──────────
    cam_centers = np.array([T[:3, 3] for T in T_base_cam_init])

    # (source_id, target_id, T_target_source, info, uncertain)
    edges: list[tuple[int, int, np.ndarray, np.ndarray, bool]] = []

    def _run_icp(i: int, j: int) -> tuple[np.ndarray, np.ndarray, float] | None:
        """source = j, target = i. init은 base→cam_init에서 유도."""
        T_init = np.linalg.inv(T_base_cam_init[i]) @ T_base_cam_init[j]
        result = o3d.pipelines.registration.registration_icp(
            source=pcds_down[j],
            target=pcds_down[i],
            max_correspondence_distance=icp_max_dist,
            init=T_init,
            estimation_method=(
                o3d.pipelines.registration.TransformationEstimationPointToPlane()
            ),
        )
        try:
            info = (
                o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                    pcds_down[j],
                    pcds_down[i],
                    icp_max_dist,
                    result.transformation,
                )
            )
        except Exception as e:
            logger.warning("information matrix 실패 (%d→%d): %s", j, i, e)
            return None
        return result.transformation, info, float(result.fitness)

    # 인접 페어 — uncertain=False
    for i in range(n - 1):
        out = _run_icp(i, i + 1)
        if out is None:
            continue
        T_ij, info, _ = out
        edges.append((i + 1, i, T_ij, info, False))

    # 비인접 loop closure 후보 — uncertain=True
    for i in range(n):
        for j in range(i + 2, n):
            dist = float(np.linalg.norm(cam_centers[i] - cam_centers[j]))
            if dist > PAIR_UNCERTAIN_DIST:
                continue
            out = _run_icp(i, j)
            if out is None:
                continue
            T_ij, info, fitness = out
            if fitness < ICP_FITNESS_FLOOR:
                continue
            edges.append((j, i, T_ij, info, True))

    # ─── 5. PoseGraph + global optimization ──────────────────────────
    pose_graph = o3d.pipelines.registration.PoseGraph()
    for T in T_base_cam_init:
        pose_graph.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(T.copy())
        )
    for src, tgt, T_ts, info, uncertain in edges:
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                source_node_id=src,
                target_node_id=tgt,
                transformation=T_ts,
                information=info,
                uncertain=uncertain,
            )
        )

    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=icp_max_dist,
        edge_prune_threshold=0.25,
        reference_node=0,
    )
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option,
    )

    T_base_cam_refined = [pose_graph.nodes[i].pose for i in range(n)]

    # ─── 6. TSDF integrate ──────────────────────────────────────────
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_size,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )
    for i in range(n):
        extrinsic = np.linalg.inv(T_base_cam_refined[i])
        volume.integrate(rgbds[i], intrinsics[i], extrinsic)

    # ─── 7. Mesh + cluster post-process ────────────────────────────
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    cluster_ids, cluster_sizes, _ = mesh.cluster_connected_triangles()
    cluster_ids = np.asarray(cluster_ids)
    cluster_sizes = np.asarray(cluster_sizes)
    if len(cluster_sizes) > 0:
        small_clusters = np.where(cluster_sizes < MIN_TRIANGLE_CLUSTER_SIZE)[0]
        if len(small_clusters) > 0:
            triangle_mask = np.isin(cluster_ids, small_clusters)
            mesh.remove_triangles_by_mask(triangle_mask)
            mesh.remove_unreferenced_vertices()

    # ─── 8. 저장 ───────────────────────────────────────────────────
    out_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(out_path), mesh, write_vertex_normals=True)

    return BuildResult(
        path=out_path,
        vertex_count=len(mesh.vertices),
        triangle_count=len(mesh.triangles),
        n_scans=n,
        n_edges=len(edges),
        refined_poses=T_base_cam_refined,
    )
