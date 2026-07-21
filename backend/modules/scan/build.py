"""TSDF reconstruction build — 옛 backend/modules/reconstruction/build.py 이월.

multi-view **multiscale colored ICP** + PoseGraph 전역 최적화 + ScalableTSDF
integration + mesh 추출 + 작은 cluster 제거. 순수 Open3D — kinematics/calibration
dep 없음 (호출자가 각 scan 의 초기 camera pose T_base_cam_init 을 미리 계산해 넘김).

정합이 colored ICP 인 이유 (2026-07-18 실측 진단, docs/perception.md): 작업대
장면은 평면이 지배하는데 point-to-plane ICP 는 평면 내 3자유도(x/y/yaw)에
퇴화한다 — 평면을 따라 미끄러져도 비용이 안 늘어서, in-plane 정렬이 FK+hand_eye
초기값(σ_t ~7.5mm) 그대로 남고 TSDF 가 그 오정합을 평균해 텍스처가 번졌다
(session 2 실측: voxel 2mm 인데 ~7mm 스케일 번짐). colored ICP (Park et al.
2017) 는 색 그라디언트 항이 그 자유도를 구속한다. 무텍스처 등으로 실패하면
point-to-plane 폴백 (register_pair).

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
# loop closure 후보 임계 (camera 중심 거리). 0.15→0.30 (2026-07-18): 원거리
# 관측(~0.3m)의 D405 발자국은 ~0.5×0.3m 라 30cm 떨어진 pose 도 중첩이 충분 —
# 실제 게이트는 fitness floor. (session 2 실측: 0.15 에서 loop edge 0개.)
_PAIR_UNCERTAIN_DIST = 0.30
_ICP_FITNESS_FLOOR = 0.3
# 정합 발산 게이트 (2026-07-18 실물 로그 튜닝) — FK 초기값 대비 ICP 보정량 상한(m).
# FK 는 σ_t ~7.5mm 신뢰라 정상 보정은 실측 ≤28mm. 그런데 큰 평면(책상) 위
# colored ICP 는 반복 텍스처(나뭇결/테이프)에 aliasing 되어 80~130mm 엉뚱한 곳에
# lock 되고, 그 오정합의 fitness(0.31~0.57)가 floor 를 넘겨 PoseGraph 를 오염시켜
# 스캔이 날아간다 (실물 recon10: pair 4→0 corr 132mm/fit 0.31). **fitness 는 평면
# aliasing 에서 못 믿는다** — FK 이탈량(corr)이 신뢰 신호. 이 상한 초과 = 발산으로
# 보고 그 edge 는 FK 초기값으로 대체 (loop 은 아예 제외). 실측 분리: 정상 ≤28mm /
# 발산 ≥51mm → 40mm.
_MAX_CORR_M = 0.04
# 저신뢰 인접 채택 게이트 (2026-07-21 실물 회귀 — docs/pnp_scenario_rework.md §8).
# 07-21 붕괴 런: 인접 fitness 0.07(corr 12mm)·0.32(38mm)·0.33(22mm) 가 전부
# "경고만 찍고 채택"돼 PoseGraph 를 오염 → 이중벽 mesh (07-20 21:58 도 0.36/39mm
# 동일 클래스). 07-19 건강 런 실측: 인접 fitness ≥0.50, corr ≤11.3mm. 그 사이에
# 문턱 2개:
#   ① fitness < _ICP_FITNESS_FLOOR: 지지 중첩 자체가 없음 — 보정량 무관 FK.
#   ② fitness < _FITNESS_TRUST 이고 corr > _CORR_SUSPECT_M: 약한 중첩이 FK
#      신뢰대역(σ_t ~7.5mm) 밖으로 크게 옮김 = 오정합 의심 — FK.
# (0.38/6.8mm 처럼 "약하지만 보정 작은" 쌍은 통과 — FK 근처 미세 보정은 무해.)
_FITNESS_TRUST = 0.45
_CORR_SUSPECT_M = 0.015
# multiscale colored ICP 사다리 (coarse→fine): (다운샘플 voxel m, max_iter).
# max_correspondence = 그 스케일 voxel — 최상단 20mm 가 FK 초기오차(σ_t
# ~7.5mm + 유격)를 덮고, fine 으로 내려가며 조인다 (Open3D 관례 스케줄).
_COLORED_SCALES: tuple[tuple[float, int], ...] = ((0.02, 50), (0.01, 30), (0.005, 14))
_MIN_CLUSTER = 500
MIN_SCANS = 2


def sdf_trunc_for(voxel_size: float) -> float:
    """voxel 에 결합된 TSDF truncation 파생 (호출자가 voxel 만 고를 때).

    band 가 얇으면(≲2 voxel) marching cubes 가 찢어져 구멍이 나고, 절대값이
    작으면 정합 잔차에 표면이 상쇄된다. 현행 기본(2mm/10mm = 5×)을 앵커로
    비율 유지 + 하한 10mm — 1/2mm 는 현행 그대로, 4mm→20mm, 8mm→40mm."""
    return max(DEFAULT_SDF_TRUNC, 5.0 * voxel_size)

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
class PairStat:
    """쌍 정합 관측치 — 병목/품질 분석용 (호출자 로그 + 오프라인 스크립트).

    corr_mm = ICP 가 FK 초기 상대자세에서 고친 이동량 — 이 값이 수 mm 급이면
    "FK 만 믿은 옛 정합은 그만큼 오정합이었다"는 직접 증거."""

    src: int
    tgt: int
    loop: bool  # loop closure 후보였나 (False = 인접 체인)
    method: str  # "colored" | "p2p_fallback" | "…→fk" (발산 → FK 대체)
    fitness: float
    corr_mm: float  # ICP 가 FK 초기값에서 옮기려던 양 (발산 탐지 신호)
    trusted: bool = True  # corr ≤ _MAX_CORR_M (발산 아님). loop edge 채택 조건


@dataclass
class BuildResult:
    mesh_bytes: bytes  # .ply
    vertex_count: int
    triangle_count: int
    n_scans: int
    n_edges: int
    elapsed: float = 0.0
    refined_poses: list[np.ndarray] = field(default_factory=list)
    # 성능 계측 (2026-07-18): stage 별 소요 ms — 병목 판정표의 입력
    # (icp 느림→pose/스케일 조정, tsdf/extract 느림→빌드 주기·증분, …).
    stage_ms: dict[str, float] = field(default_factory=dict)
    pairs: list[PairStat] = field(default_factory=list)


def build_pyramid(
    pcd: o3d.geometry.PointCloud,
    scales: tuple[tuple[float, int], ...] = _COLORED_SCALES,
) -> dict[float, o3d.geometry.PointCloud]:
    """정합용 스케일 피라미드 — voxel 다운샘플 + 법선 (colored ICP 요건).

    키 = 스케일 voxel(m). min(키) = 최세밀 레벨 (info matrix/p2p 폴백 공용)."""
    levels: dict[float, o3d.geometry.PointCloud] = {}
    for v, _ in scales:
        d = pcd.voxel_down_sample(v)
        d.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=v * 2.0, max_nn=30)
        )
        # 법선 방향 일관화 — 점군은 camera frame (원점 = 카메라). estimate 는
        # 부호가 임의라 colored ICP tangent/p2p 부호 일관성을 위해 정렬.
        d.orient_normals_towards_camera_location()
        levels[v] = d
    return levels


def pair_gate(
    fitness: float, corr_m: float, max_corr_m: float = _MAX_CORR_M
) -> tuple[bool, str]:
    """쌍 정합 채택 판정 (순수 함수 — 회귀 테스트 대상).

    반환 = (trusted, 기각 사유). trusted=False 면 호출자(register_pair)가 T 를
    FK 초기값으로 되돌린다. 사유는 method 접미로 로그에 남는다 (2026-07-21
    붕괴 로그가 "왜 기각인지" 없이 채택만 찍혀 진단이 늦었던 것의 관측성 보강)."""
    if corr_m > max_corr_m:
        return False, "발산"
    if fitness < _ICP_FITNESS_FLOOR:
        return False, "저fitness"
    if fitness < _FITNESS_TRUST and corr_m > _CORR_SUSPECT_M:
        return False, "약중첩+대보정"
    return True, ""


def register_pair(
    levels_src: dict[float, o3d.geometry.PointCloud],
    levels_tgt: dict[float, o3d.geometry.PointCloud],
    t_ini: np.ndarray,
    *,
    icp_max_dist: float = DEFAULT_ICP_MAX_DIST,
    max_corr_m: float = _MAX_CORR_M,
    scales: tuple[tuple[float, int], ...] = _COLORED_SCALES,
) -> tuple[np.ndarray, np.ndarray, float, str, float, bool]:
    """쌍 정합: multiscale colored ICP, 실패/저품질 시 point-to-plane 폴백,
    **과대 보정(발산) 시 FK 초기값으로 대체**.

    colored 채택 근거는 모듈 docstring (평면 지배 장면의 in-plane 퇴화).
    폴백 경로 = 옛 파이프라인과 동일 알고리즘 (무텍스처/무중첩에서 colored 는
    수렴 실패·예외가 날 수 있다 — mock 카메라의 무텍스처 프레임 포함).

    발산 게이트 (_MAX_CORR_M): colored/p2p 가 FK 초기값에서 max_corr_m 넘게
    움직였으면 평면 aliasing lock 으로 보고 (fitness 는 그 상황서 못 믿음) T 를
    FK 초기값으로 되돌린다. 반환 trusted=False → 호출부가 loop edge 를 버린다
    (인접 edge 는 FK 로라도 연결성 유지). 반환 = (T, info, fitness, method,
    corr_m, trusted). corr_m = ICP 가 옮기려던 양 (되돌리기 전 — 진단 신호)."""
    fine = min(levels_src)  # 최세밀 스케일 voxel
    t_cur = np.asarray(t_ini, dtype=float)
    fitness = 0.0
    method = "colored"
    try:
        for v, iters in scales:
            res = o3d.pipelines.registration.registration_colored_icp(
                levels_src[v],
                levels_tgt[v],
                v,  # max_correspondence = 스케일 voxel (Open3D 관례)
                t_cur,
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=iters
                ),
            )
            t_cur = res.transformation
            fitness = float(res.fitness)
        if fitness < _ICP_FITNESS_FLOOR:
            raise RuntimeError(f"colored fitness {fitness:.2f} < floor")
    except Exception:
        method = "p2p_fallback"
        res = o3d.pipelines.registration.registration_icp(
            levels_src[fine],
            levels_tgt[fine],
            icp_max_dist,
            np.asarray(t_ini, dtype=float),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )
        t_cur = res.transformation
        fitness = float(res.fitness)
    # 채택 게이트 (pair_gate) — 발산(corr 상한) + 저fitness + 약중첩·대보정 콤보.
    # 기각 시 FK 초기값으로 되돌린다 (fitness 는 평면/반복 텍스처서 못 믿음).
    t_cur = np.asarray(t_cur, dtype=float)
    t_ini_arr = np.asarray(t_ini, dtype=float)
    corr_m = float(np.linalg.norm(t_cur[:3, 3] - t_ini_arr[:3, 3]))
    trusted, reason = pair_gate(fitness, corr_m, max_corr_m)
    if not trusted:
        method = f"{method}→fk({reason})"
        t_cur = t_ini_arr
    info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        levels_src[fine], levels_tgt[fine], icp_max_dist, t_cur
    )
    return t_cur, info, fitness, method, corr_m, trusted


def build_mesh(
    scans: list[BuildScanInput],
    *,
    voxel_size: float = DEFAULT_VOXEL_SIZE,
    sdf_trunc: float = DEFAULT_SDF_TRUNC,
    depth_trunc: float = DEFAULT_DEPTH_TRUNC,
    icp_max_dist: float = DEFAULT_ICP_MAX_DIST,
    roi: tuple[float, float, float, float, float, float] | None = None,
    progress: ProgressCallback = _noop,
) -> BuildResult:
    if len(scans) < MIN_SCANS:
        raise ValueError(f"build_mesh: scan {len(scans)}개 < 최소 {MIN_SCANS}")
    t0 = time.time()
    n = len(scans)

    # stage 별 소요 계측 — "한 런 후 로그만으로 병목 판정" 안전망 (BuildResult).
    stage_ms: dict[str, float] = {}
    t_stage = time.perf_counter()

    def _lap(name: str) -> None:
        nonlocal t_stage
        now = time.perf_counter()
        stage_ms[name] = round((now - t_stage) * 1000.0, 1)
        t_stage = now

    # ── STAGE 1: load + RGBD ──────────────────────────────────
    progress("loading_scans", 0.0, "scan 로딩")
    rgbds: list[o3d.geometry.RGBDImage] = []
    intrinsics: list[o3d.camera.PinholeCameraIntrinsic] = []
    pcds: list[o3d.geometry.PointCloud] = []
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
        pcds.append(o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intr))
        progress("loading_scans", (i + 1) / n, f"scan {i + 1}/{n}")
    _lap("load")

    # ICP 용 스케일 피라미드 (TSDF voxel 과 독립 — voxel 1mm 를 골라도 정합
    # 비용은 불변). 최세밀 레벨은 info matrix/폴백에도 쓴다.
    progress("loading_scans", 1.0, "pyramid 구성")
    pyramids = [build_pyramid(p) for p in pcds]
    _lap("pyramid")

    # ── STAGE 2: pairwise 정합 (인접 체인 + loop closure) ─────
    progress("pairwise_registration", 0.0, "ICP 정합")
    edges: list[tuple[int, int, np.ndarray, np.ndarray, bool]] = []
    pairs: list[PairStat] = []

    def _register(
        src: int, tgt: int, loop: bool
    ) -> tuple[np.ndarray, np.ndarray, float, bool]:
        t_ini = np.linalg.inv(t_init[tgt]) @ t_init[src]
        t_icp, info, fitness, method, corr_m, trusted = register_pair(
            pyramids[src], pyramids[tgt], t_ini, icp_max_dist=icp_max_dist
        )
        pairs.append(
            PairStat(
                src=src, tgt=tgt, loop=loop, method=method,
                fitness=fitness, corr_mm=corr_m * 1000.0, trusted=trusted,
            )
        )
        logger.info(
            "  pair %d→%d%s: %s fitness=%.2f, FK 대비 보정 %.1fmm%s",
            src, tgt, " (loop)" if loop else "", method, fitness, corr_m * 1000.0,
            "" if trusted else " ⚠ 기각→FK",
        )
        return t_icp, info, fitness, trusted

    for i in range(n - 1):
        t_ts, info, fit, trusted = _register(i + 1, i, loop=False)
        if not trusted:
            # 인접 기각 = FK 초기값으로 대체됨 (register_pair.pair_gate — 사유는
            # 위 pair 로그의 method 접미). 연결성은 FK edge 로 유지 — node init 도
            # FK 라 왜곡 없음. 그 스캔은 FK(~7.5mm)로 배치 — 07-21 실측: 오정합
            # 38mm 채택보다 FK 배치가 낫다 (docs/pnp_scenario_rework.md §8).
            logger.warning(
                "  인접 pair %d→%d 정합 기각 — FK 초기값으로 배치 (fitness %.2f)",
                i + 1, i, fit,
            )
        edges.append((i + 1, i, t_ts, info, False))
        progress("pairwise_registration", (i + 1) / max(1, n - 1), f"인접 {i + 1}")

    centers = np.array([t[:3, 3] for t in t_init])
    n_loop_cand = 0
    n_loop_kept = 0
    for i in range(n):
        for j in range(i + 2, n):
            dist = float(np.linalg.norm(centers[i] - centers[j]))
            if dist > _PAIR_UNCERTAIN_DIST:
                continue
            n_loop_cand += 1
            t_ts, info, fit, trusted = _register(j, i, loop=True)
            # loop edge 는 정보 추가가 목적 — 게이트 기각(발산/저fitness/약중첩·
            # 대보정, pair_gate)이면 node init 대비 얻는 게 없고 오염 위험 → 제외.
            if not trusted:
                logger.info("  loop 후보 %d→%d 기각 — 제외 (FK 이상 정보 없음)", j, i)
                continue
            n_loop_kept += 1
            edges.append((j, i, t_ts, info, True))
    logger.info(
        "pairwise 완료: 인접 %d + loop %d/%d (후보 임계 %.2fm)",
        n - 1, n_loop_kept, n_loop_cand, _PAIR_UNCERTAIN_DIST,
    )
    _lap("icp")

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
    _lap("graph")

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
    _lap("tsdf")

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

    # ROI 크롭 — 최종 mesh 를 작업 셀 상자 안으로 (base frame AABB). 정합(ICP)은
    # 전체 클라우드로 이미 끝났으므로(원거리 특징 = 회전 앵커 유지) 여기서만 잘라
    # 침구/바닥/가방 등 셀 밖을 mesh 에서 제거 (docs/pnp_scenario_rework.md §3.3).
    # 관측성: 크롭 전/후 정점 수 로그 (집에서 "왜 이만큼 잘렸나" 진단 데이터).
    if roi is not None:
        x0, x1, y0, y1, z0, z1 = roi
        n_before = len(mesh.vertices)
        aabb = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(x0, y0, z0), max_bound=(x1, y1, z1)
        )
        mesh = mesh.crop(aabb)
        logger.info(
            "ROI 크롭: 정점 %d → %d (셀 x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f])",
            n_before, len(mesh.vertices), x0, x1, y0, y1, z0, z1,
        )
    _lap("extract")

    mesh_bytes = _mesh_to_ply_bytes(mesh)
    progress("mesh_extraction", 1.0, "mesh 완료")
    _lap("encode")

    elapsed = time.time() - t0
    logger.info(
        "build_mesh done: %d scans, %d edges, %d verts, %d tris, %.1fs | %s",
        n, len(edges), len(mesh.vertices), len(mesh.triangles), elapsed,
        " ".join(f"{k}={v:.0f}ms" for k, v in stage_ms.items()),
    )
    return BuildResult(
        mesh_bytes=mesh_bytes,
        vertex_count=len(mesh.vertices),
        triangle_count=len(mesh.triangles),
        n_scans=n,
        n_edges=len(edges),
        elapsed=elapsed,
        refined_poses=t_refined,
        stage_ms=stage_ms,
        pairs=pairs,
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
