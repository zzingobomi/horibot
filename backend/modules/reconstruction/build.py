"""Multi-view reconstruction — ICP + multi-way PoseGraph + TSDF + mesh extract.

이전 [modules/pointcloud/tsdf_builder.py](../pointcloud/tsdf_builder.py) 자리 자체
자리 이동 자리 + 변경:
- 입력: BuildScanInput list (storage 의 ScanRecord + blob decode 결과 자리)
- 출력: BuildResult (mesh_bytes + metadata, path 자리 제거 — storage put 자리)
- progress_callback 자리 추가 — 5 stage publish 자리

흐름 (storage_layer.md §3 Phase 2 + reconstruction.md):
  1. 자세별 init pose: raw_motor → JointCoordinates → Kinematics.fk → T_base_cam_init
  2. depth bilateral filter (edge 보존 + stereo 노이즈 ↓)
  3. RGBD → PointCloud (cam frame) + normal 추정
  4. pair-wise ICP (인접 + loop closure)
  5. PoseGraph + global_optimization (Levenberg-Marquardt)
  6. ScalableTSDFVolume.integrate
  7. extract_triangle_mesh + cluster filter
  8. PLY bytes 반환 (tempfile 자리 우회)
"""

from __future__ import annotations

import logging
import os
import tempfile
from dataclasses import dataclass, field
from typing import Callable

import cv2
import numpy as np
import open3d as o3d

from core.coords.joint_coordinates import JointCoordinates
from modules.calibration.calibration_cache import CalibrationCache
from modules.motor.motor_config import MotorConfig

logger = logging.getLogger(__name__)

# ─── 디폴트 (이전 tsdf_builder.py 자리 자리 그대로) ───
DEFAULT_VOXEL_SIZE = 0.002  # 2mm
DEFAULT_SDF_TRUNC = 0.010  # 10mm
DEFAULT_DEPTH_TRUNC = 0.5  # m — D405 sweet spot
DEFAULT_ICP_MAX_DIST = 0.010  # m
DEFAULT_BILATERAL_DIAMETER = 5
DEFAULT_BILATERAL_SIGMA_COLOR = 50.0
DEFAULT_BILATERAL_SIGMA_SPACE = 50.0
PAIR_UNCERTAIN_DIST = 0.15  # m — loop closure 후보 거리 임계
ICP_FITNESS_FLOOR = 0.3
MIN_TRIANGLE_CLUSTER_SIZE = 500
MIN_SCANS = 2

# stage 자리 (reconstruction message 의 Literal 과 정합)
STAGE_LOADING = "loading_scans"
STAGE_PAIRWISE = "pairwise_registration"
STAGE_POSE_GRAPH = "pose_graph_optimization"
STAGE_TSDF = "tsdf_integration"
STAGE_MESH = "mesh_extraction"

ProgressCallback = Callable[[str, float, str], None]


@dataclass
class BuildScanInput:
    """build_mesh 자리 입력 — storage ScanRecord + blob decode 결과 합쳐 caller 가 생성."""

    color_bgr: np.ndarray  # H x W x 3 uint8
    depth_z16: np.ndarray  # H x W uint16
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float
    motor_positions: list[int]
    arm_motor_ids: list[int]


@dataclass
class BuildResult:
    mesh_bytes: bytes
    vertex_count: int
    triangle_count: int
    n_scans: int
    n_edges: int
    elapsed: float = 0.0
    refined_poses: list[np.ndarray] = field(default_factory=list)


def _noop_progress(stage: str, percent: float, message: str) -> None:
    pass


def build_mesh(
    scans: list[BuildScanInput],
    arm_cfgs: list[MotorConfig],
    *,
    robot_id: str,
    voxel_size: float = DEFAULT_VOXEL_SIZE,
    sdf_trunc: float = DEFAULT_SDF_TRUNC,
    depth_trunc: float = DEFAULT_DEPTH_TRUNC,
    icp_max_dist: float = DEFAULT_ICP_MAX_DIST,
    bilateral_diameter: int = DEFAULT_BILATERAL_DIAMETER,
    progress: ProgressCallback = _noop_progress,
) -> BuildResult:
    import time as _time

    if len(scans) < MIN_SCANS:
        raise ValueError(f"scan {MIN_SCANS}개 이상 필요. 현재: {len(scans)}")

    progress(STAGE_LOADING, 0.0, f"{len(scans)} scan 준비")

    calib = CalibrationCache().get(robot_id)
    if calib.hand_eye is None:
        raise RuntimeError("hand_eye 캘 없음 — 캘리브 먼저 진행")

    T_ee_cam = np.eye(4)
    T_ee_cam[:3, :3] = calib.hand_eye.R
    T_ee_cam[:3, 3] = calib.hand_eye.t.reshape(3)

    from core.robot.robot_registry import RobotRegistry
    from modules.kinematics.adapters.sag_corrected import SagCorrectedKinematics

    kinematics_obj = RobotRegistry().get_kinematics(robot_id)
    assert isinstance(kinematics_obj, SagCorrectedKinematics)
    kinematics = kinematics_obj
    coords = JointCoordinates()
    cfg_by_id = {cfg.id: cfg for cfg in arm_cfgs}

    t0 = _time.time()

    T_base_cam_init: list[np.ndarray] = []
    rgbds: list[o3d.geometry.RGBDImage] = []
    intrinsics: list[o3d.camera.PinholeCameraIntrinsic] = []
    pcds_down: list[o3d.geometry.PointCloud] = []

    # ─── 1~3. 자세별 init pose + RGBD + cloud + normal ──────────────
    for idx, s in enumerate(scans):
        arm_rad: list[float] = []
        for raw, mid in zip(s.motor_positions, s.arm_motor_ids):
            cfg = cfg_by_id.get(int(mid))
            if cfg is None:
                raise RuntimeError(
                    f"scan #{idx} motor id {mid}가 현재 arm_cfgs에 없음"
                )
            arm_rad.append(coords.motor_to_urdf(int(raw), cfg, robot_id=robot_id))

        R_be, t_be = kinematics.fk_to_matrix(arm_rad)
        T_base_ee = np.eye(4)
        T_base_ee[:3, :3] = np.asarray(R_be)
        T_base_ee[:3, 3] = np.asarray(t_be)
        T_bc = T_base_ee @ T_ee_cam
        T_base_cam_init.append(T_bc)

        depth_f = s.depth_z16.astype(np.float32)
        depth_filtered = cv2.bilateralFilter(
            depth_f,
            bilateral_diameter,
            DEFAULT_BILATERAL_SIGMA_COLOR,
            DEFAULT_BILATERAL_SIGMA_SPACE,
        ).astype(np.uint16)

        color_rgb = np.ascontiguousarray(s.color_bgr[:, :, ::-1])
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_rgb),
            o3d.geometry.Image(depth_filtered),
            depth_scale=1.0 / s.depth_scale,
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False,
        )
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            s.width, s.height, s.fx, s.fy, s.cx, s.cy
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
        progress(STAGE_LOADING, (idx + 1) / len(scans), f"scan {idx + 1}/{len(scans)} 준비")

    n = len(scans)
    progress(STAGE_LOADING, 1.0, f"{n} scan 로드 완료")

    # ─── 4. Pair-wise ICP ────────────────────────────────────
    cam_centers = np.array([T[:3, 3] for T in T_base_cam_init])
    edges: list[tuple[int, int, np.ndarray, np.ndarray, bool]] = []

    def _run_icp(i: int, j: int):
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

    # 인접 페어 + loop closure 후보
    total_pairs = (n - 1)
    for i in range(n - 1):
        out = _run_icp(i, i + 1)
        progress(
            STAGE_PAIRWISE,
            (i + 1) / max(total_pairs, 1),
            f"인접 ICP {i + 1}/{total_pairs}",
        )
        if out is None:
            continue
        T_ij, info, _ = out
        edges.append((i + 1, i, T_ij, info, False))

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
    progress(STAGE_PAIRWISE, 1.0, f"ICP edges={len(edges)}")

    # ─── 5. PoseGraph + global optimization ──────────────────
    progress(STAGE_POSE_GRAPH, 0.0, "pose graph 최적화")
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
    progress(STAGE_POSE_GRAPH, 1.0, "pose graph 완료")

    # ─── 6. TSDF integrate ──────────────────────────────────
    progress(STAGE_TSDF, 0.0, "TSDF volume 적분")
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_size,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )
    for i in range(n):
        extrinsic = np.linalg.inv(T_base_cam_refined[i])
        volume.integrate(rgbds[i], intrinsics[i], extrinsic)
        progress(STAGE_TSDF, (i + 1) / n, f"TSDF {i + 1}/{n}")

    # ─── 7. Mesh + cluster post-process ─────────────────────
    progress(STAGE_MESH, 0.0, "mesh extract")
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
    progress(STAGE_MESH, 0.7, "cluster filter")

    # ─── 8. PLY bytes ───────────────────────────────────────
    # open3d 의 write_triangle_mesh 자리 path-only — tempfile 자리 read 우회.
    fd, temp_path = tempfile.mkstemp(suffix=".ply", prefix="horibot_recon_")
    try:
        os.close(fd)
        o3d.io.write_triangle_mesh(temp_path, mesh, write_vertex_normals=True)
        with open(temp_path, "rb") as f:
            mesh_bytes = f.read()
    finally:
        try:
            os.unlink(temp_path)
        except OSError:
            pass
    progress(STAGE_MESH, 1.0, f"mesh: {len(mesh.vertices)} verts")

    elapsed = _time.time() - t0
    return BuildResult(
        mesh_bytes=mesh_bytes,
        vertex_count=len(mesh.vertices),
        triangle_count=len(mesh.triangles),
        n_scans=n,
        n_edges=len(edges),
        elapsed=elapsed,
        refined_poses=T_base_cam_refined,
    )
