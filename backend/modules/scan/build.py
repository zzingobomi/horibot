"""TSDF reconstruction build — 옛 backend/modules/reconstruction/build.py 이월.

multi-view ICP + PoseGraph 전역 최적화 + ScalableTSDF integration + mesh 추출 +
작은 cluster 제거. 순수 Open3D — kinematics/calibration dep 없음 (호출자가 각 scan 의
초기 camera pose T_base_cam_init 을 미리 계산해 넘김).

CRITICAL gotcha 이월:
- depth_scale 은 create_from_color_and_depth 에 역수 (1/depth_scale).
- BGR → RGB.
- TSDF integrate extrinsic = inv(T_base_cam) (camera→base).
"""

from __future__ import annotations

import logging
import os
import tempfile
import time
from dataclasses import dataclass, field
from typing import Callable

import cv2
import numpy as np
import open3d as o3d

logger = logging.getLogger(__name__)

DEFAULT_VOXEL_SIZE = 0.002  # 2mm
DEFAULT_SDF_TRUNC = 0.010  # 10mm
DEFAULT_DEPTH_TRUNC = 0.5  # m (D405 sweet spot)
DEFAULT_ICP_MAX_DIST = 0.010  # 10mm
_BILATERAL_D = 5
_BILATERAL_SIGMA_COLOR = 50.0
_BILATERAL_SIGMA_SPACE = 50.0
_PAIR_UNCERTAIN_DIST = 0.15  # m (loop closure 후보 임계)
_ICP_FITNESS_FLOOR = 0.3
_MIN_CLUSTER = 500
MIN_SCANS = 2

ProgressCallback = Callable[[str, float, str], None]


def _noop(stage: str, percent: float, message: str) -> None:
    pass


@dataclass
class BuildScanInput:
    color_bgr: np.ndarray  # HxWx3 uint8
    depth_z16: np.ndarray  # HxW uint16
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float
    t_base_cam_init: np.ndarray  # 4x4 초기 camera pose (base frame)


@dataclass
class BuildResult:
    mesh_bytes: bytes  # .ply
    vertex_count: int
    triangle_count: int
    n_scans: int
    n_edges: int
    elapsed: float = 0.0
    refined_poses: list[np.ndarray] = field(default_factory=list)


def build_mesh(
    scans: list[BuildScanInput],
    *,
    voxel_size: float = DEFAULT_VOXEL_SIZE,
    sdf_trunc: float = DEFAULT_SDF_TRUNC,
    depth_trunc: float = DEFAULT_DEPTH_TRUNC,
    icp_max_dist: float = DEFAULT_ICP_MAX_DIST,
    progress: ProgressCallback = _noop,
) -> BuildResult:
    if len(scans) < MIN_SCANS:
        raise ValueError(f"build_mesh: scan {len(scans)}개 < 최소 {MIN_SCANS}")
    t0 = time.time()
    n = len(scans)

    # ── STAGE 1: load + RGBD + normal ─────────────────────────
    progress("loading_scans", 0.0, "scan 로딩")
    rgbds: list[o3d.geometry.RGBDImage] = []
    intrinsics: list[o3d.camera.PinholeCameraIntrinsic] = []
    pcds_down: list[o3d.geometry.PointCloud] = []
    t_init = [s.t_base_cam_init for s in scans]

    for i, s in enumerate(scans):
        depth_f = s.depth_z16.astype(np.float32)
        depth_filtered = cv2.bilateralFilter(
            depth_f, _BILATERAL_D, _BILATERAL_SIGMA_COLOR, _BILATERAL_SIGMA_SPACE
        ).astype(np.uint16)
        color_rgb = np.ascontiguousarray(s.color_bgr[:, :, ::-1])
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_rgb),
            o3d.geometry.Image(depth_filtered),
            depth_scale=1.0 / s.depth_scale,
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False,
        )
        rgbds.append(rgbd)
        intr = o3d.camera.PinholeCameraIntrinsic(
            s.width, s.height, s.fx, s.fy, s.cx, s.cy
        )
        intrinsics.append(intr)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intr)
        pcd_d = pcd.voxel_down_sample(voxel_size)
        pcd_d.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30)
        )
        pcds_down.append(pcd_d)
        progress("loading_scans", (i + 1) / n, f"scan {i + 1}/{n}")

    # ── STAGE 2: pairwise ICP (adjacent + loop closure) ───────
    progress("pairwise_registration", 0.0, "ICP 정합")
    edges: list[tuple[int, int, np.ndarray, np.ndarray, bool]] = []
    estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()

    def _icp(src: int, tgt: int) -> tuple[np.ndarray, np.ndarray, float]:
        t_ini = np.linalg.inv(t_init[tgt]) @ t_init[src]
        res = o3d.pipelines.registration.registration_icp(
            pcds_down[src], pcds_down[tgt], icp_max_dist, t_ini, estimation
        )
        info = (
            o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                pcds_down[src], pcds_down[tgt], icp_max_dist, res.transformation
            )
        )
        return res.transformation, info, float(res.fitness)

    for i in range(n - 1):
        t_ts, info, _fit = _icp(i + 1, i)
        edges.append((i + 1, i, t_ts, info, False))
        progress("pairwise_registration", (i + 1) / max(1, n - 1), f"인접 {i + 1}")

    centers = np.array([t[:3, 3] for t in t_init])
    for i in range(n):
        for j in range(i + 2, n):
            if float(np.linalg.norm(centers[i] - centers[j])) > _PAIR_UNCERTAIN_DIST:
                continue
            t_ts, info, fit = _icp(j, i)
            if fit < _ICP_FITNESS_FLOOR:
                continue
            edges.append((j, i, t_ts, info, True))

    # ── STAGE 3: PoseGraph 전역 최적화 ────────────────────────
    progress("pose_graph_optimization", 0.0, "pose graph 최적화")
    pose_graph = o3d.pipelines.registration.PoseGraph()
    for t in t_init:
        pose_graph.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(t.copy())
        )
    for src, tgt, t_ts, info, uncertain in edges:
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                source_node_id=src,
                target_node_id=tgt,
                transformation=t_ts,
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
    t_refined = [pose_graph.nodes[i].pose for i in range(n)]
    progress("pose_graph_optimization", 1.0, "최적화 완료")

    # ── STAGE 4: TSDF integration ─────────────────────────────
    progress("tsdf_integration", 0.0, "TSDF 통합")
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_size,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )
    for i in range(n):
        extrinsic = np.linalg.inv(t_refined[i])  # camera→base
        volume.integrate(rgbds[i], intrinsics[i], extrinsic)
        progress("tsdf_integration", (i + 1) / n, f"통합 {i + 1}/{n}")

    # ── STAGE 5: mesh 추출 + cluster 필터 ─────────────────────
    progress("mesh_extraction", 0.0, "mesh 추출")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    cluster_ids, cluster_sizes, _ = mesh.cluster_connected_triangles()
    cluster_ids = np.asarray(cluster_ids)
    cluster_sizes = np.asarray(cluster_sizes)
    if len(cluster_sizes) > 0:
        small = np.where(cluster_sizes < _MIN_CLUSTER)[0]
        if len(small) > 0:
            mask = np.isin(cluster_ids, small)
            mesh.remove_triangles_by_mask(mask)
            mesh.remove_unreferenced_vertices()

    mesh_bytes = _mesh_to_ply_bytes(mesh)
    progress("mesh_extraction", 1.0, "mesh 완료")

    elapsed = time.time() - t0
    logger.info(
        "build_mesh done: %d scans, %d edges, %d verts, %d tris, %.1fs",
        n, len(edges), len(mesh.vertices), len(mesh.triangles), elapsed,
    )
    return BuildResult(
        mesh_bytes=mesh_bytes,
        vertex_count=len(mesh.vertices),
        triangle_count=len(mesh.triangles),
        n_scans=n,
        n_edges=len(edges),
        elapsed=elapsed,
        refined_poses=t_refined,
    )


def _mesh_to_ply_bytes(mesh: o3d.geometry.TriangleMesh) -> bytes:
    """o3d 는 bytes write 미지원 → tempfile 경유 (옛 backend 동일)."""
    fd, path = tempfile.mkstemp(suffix=".ply", prefix="horibot_recon_")
    try:
        os.close(fd)
        o3d.io.write_triangle_mesh(path, mesh, write_vertex_normals=True)
        with open(path, "rb") as f:
            return f.read()
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass
