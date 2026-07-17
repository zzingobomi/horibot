"""geometry.obb_from_base_points 단위테스트 (결정적, cv2 만).

의미 있는 검증: 알려진 yaw/크기로 만든 회전 사각형 base 점군에서 OBB(yaw, footprint)를
복원. minAreaRect angle 필드가 아니라 boxPoints 긴 변으로 yaw 를 뽑는 규약이 회전을
정확히 되살리는지 — 규약을 되돌리면(예: 짧은 변 사용) 즉시 깨짐.
"""

from __future__ import annotations

import math

import numpy as np

from modules.detector.geometry import (
    Obb,
    mask_contour,
    obb_corners,
    obb_from_base_points,
    top_face_points,
)


def _rect_points(
    cx: float, cy: float, long: float, short: float, yaw: float, n: int = 40
) -> np.ndarray:
    """(cx,cy) 중심, 긴 변=long(local X)/짧은 변=short(local Y), yaw 회전한 격자 점군."""
    xs = np.linspace(-long / 2, long / 2, n)
    ys = np.linspace(-short / 2, short / 2, n)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.stack([gx.ravel(), gy.ravel()], axis=1)
    rot = np.array(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    return pts @ rot.T + np.array([cx, cy])


def test_obb_recovers_axis_aligned():
    obb = obb_from_base_points(_rect_points(0.5, -0.2, 0.10, 0.04, 0.0))
    assert obb is not None
    assert abs(obb.center_xy[0] - 0.5) < 2e-3, obb.center_xy
    assert abs(obb.center_xy[1] + 0.2) < 2e-3, obb.center_xy
    assert abs(obb.footprint[0] - 0.10) < 3e-3, obb.footprint
    assert abs(obb.footprint[1] - 0.04) < 3e-3, obb.footprint
    assert abs(obb.yaw_rad) < math.radians(2), obb.yaw_rad


def test_obb_recovers_rotation():
    yaw = math.radians(30)
    obb = obb_from_base_points(_rect_points(0.0, 0.0, 0.12, 0.05, yaw))
    assert obb is not None
    # footprint 는 (긴 변, 짧은 변) — 회전 무관하게 실제 변 길이 복원
    assert abs(obb.footprint[0] - 0.12) < 3e-3, obb.footprint
    assert abs(obb.footprint[1] - 0.05) < 3e-3, obb.footprint
    assert abs(obb.yaw_rad - yaw) < math.radians(2), obb.yaw_rad


def test_obb_yaw_wraps_into_symmetric_range():
    # 80° 사각형 → 사각형 180° 대칭이라 [-π/2, π/2) 로 wrap.
    obb = obb_from_base_points(_rect_points(0.0, 0.0, 0.12, 0.05, math.radians(80)))
    assert obb is not None
    assert -math.pi / 2 <= obb.yaw_rad < math.pi / 2, obb.yaw_rad
    assert abs(obb.yaw_rad - math.radians(80)) < math.radians(2), obb.yaw_rad


def test_obb_too_few_points_returns_none():
    assert obb_from_base_points(None) is None
    assert obb_from_base_points(np.zeros((2, 2))) is None


def test_top_face_band_rejects_side_contamination():
    """윗면 band 필터 = OBB 오염 제거 회귀 (2026-07-09 실물 — 사선 샷 OBB skew).

    윗면(z=0.10, 30° 회전 사각형) + 옆면/배경 bleed 시뮬(z≈0, 한쪽으로 삐져나간
    점들). 전체 점군 OBB 는 footprint 부풀고 yaw 비틀림 — top_face_points 거치면
    윗면 진짜 footprint/yaw 복원. 필터를 빼면 dirty assert 로 즉시 잡힘.
    """
    yaw = math.radians(30)
    top_xy = _rect_points(0.0, 0.0, 0.10, 0.05, yaw)
    top = np.column_stack([top_xy, np.full(len(top_xy), 0.10)])
    # 테이블 높이(z=0) bleed — 물체 옆으로 삐져나간 오염
    side_xy = _rect_points(0.08, -0.06, 0.08, 0.04, 0.0)
    side = np.column_stack([side_xy, np.zeros(len(side_xy))])
    pts = np.vstack([top, side])

    dirty = obb_from_base_points(pts)  # 필터 없이 = 오염된 OBB
    assert dirty is not None
    assert dirty.footprint[0] > 0.12, dirty.footprint

    clean = obb_from_base_points(top_face_points(pts))
    assert clean is not None
    assert abs(clean.yaw_rad - yaw) < math.radians(2), clean.yaw_rad
    assert abs(clean.footprint[0] - 0.10) < 3e-3, clean.footprint
    assert abs(clean.footprint[1] - 0.05) < 3e-3, clean.footprint


def test_top_face_points_passthrough_and_none():
    assert top_face_points(None) is None
    xy = np.zeros((5, 2))  # z 열 없음 → 그대로 통과
    assert top_face_points(xy) is xy


def test_obb_corners_axis_aligned():
    # yaw=0, 중심 (1,2), long=0.2(x)/short=0.1(y), 평면 z=0.5.
    obb = Obb(center_xy=(1.0, 2.0), footprint=(0.2, 0.1), yaw_rad=0.0)
    c = obb_corners(obb, z=0.5)
    assert c.shape == (4, 3)
    assert np.allclose(c[:, 2], 0.5)  # 전부 평면 z
    # x 범위 = 1±0.1, y 범위 = 2±0.05
    assert abs(c[:, 0].min() - 0.9) < 1e-9 and abs(c[:, 0].max() - 1.1) < 1e-9
    assert abs(c[:, 1].min() - 1.95) < 1e-9 and abs(c[:, 1].max() - 2.05) < 1e-9


def test_obb_corners_rotated_90_swaps_extent():
    # yaw=90° → 긴 변(x)이 y 로. x 범위=±short/2, y 범위=±long/2.
    obb = Obb(center_xy=(0.0, 0.0), footprint=(0.2, 0.1), yaw_rad=math.pi / 2)
    c = obb_corners(obb, z=0.0)
    assert abs(c[:, 0].max() - 0.05) < 1e-9, c  # short/2
    assert abs(c[:, 1].max() - 0.1) < 1e-9, c  # long/2


def test_mask_contour_rectangle():
    mask = np.zeros((100, 100), dtype=bool)
    mask[20:60, 30:80] = True  # 40×50 사각형
    poly = mask_contour(mask)
    assert poly is not None and len(poly) == 4, poly  # 사각형 → 코너 4
    xs, ys = poly[:, 0], poly[:, 1]
    assert 29 <= xs.min() <= 31 and 78 <= xs.max() <= 80, poly
    assert 19 <= ys.min() <= 21 and 58 <= ys.max() <= 60, poly


def test_mask_contour_empty_none():
    assert mask_contour(np.zeros((10, 10), dtype=bool)) is None


# ─── object-centric 기하 (2026-07-14 재설계 — floor ring 추정 폐기) ──


def _cube_view_points(
    cx: float, cy: float, top_z: float, height: float, *, side: str | None,
    edge: float = 0.02, n: int = 12,
) -> np.ndarray:
    """한 뷰에서 보이는 큐브 점군 합성 — 윗면 + (보이면) 옆면 1개.

    단일 뷰의 물리 한계(가려진 면은 depth 없음)를 그대로 흉내: side=None 이면
    윗면만 (top-down), side="x" 면 +x 옆면이 보임 (사선 뷰).
    """
    xs = np.linspace(-edge / 2, edge / 2, n)
    gx, gy = np.meshgrid(xs, xs)
    top = np.stack(
        [gx.ravel() + cx, gy.ravel() + cy, np.full(n * n, top_z)], axis=1
    )
    if side is None:
        return top
    zs = np.linspace(top_z - height, top_z, n)
    gy2, gz = np.meshgrid(xs, zs)
    face = np.stack(
        [np.full(n * n, cx + edge / 2), gy2.ravel() + cy, gz.ravel()], axis=1
    )
    return np.vstack([top, face])


def test_object_metrics_single_view_underestimates_height():
    """단일 top-down 뷰(윗면만)는 height≈0 — 옆면 depth 부재의 물리 한계를
    기하가 정직하게 반영해야 (여기서 바닥을 '추측'해 height 를 만들면 회귀 —
    그 추측이 옛 floor ring 이고 phantom −0.23m 사고의 뿌리)."""
    from modules.detector.geometry import object_metrics_from_points

    pts = _cube_view_points(0.2, 0.1, top_z=-0.022, height=0.023, side=None)
    m = object_metrics_from_points(pts)
    assert m is not None
    position, base_z, height = m
    assert abs(position[0] - 0.2) < 1e-3 and abs(position[1] - 0.1) < 1e-3
    assert abs(position[2] + 0.022) < 1e-3
    assert height < 0.005  # 윗면만 → 두께 없음 (과소가 정직)


def test_object_metrics_fused_views_recover_height():
    """멀티뷰 융합(윗면 + 옆면)이 실 height/base_z 를 복원 — floor 추정 없이
    물체 자기 점군만으로. grasp_redesign_journey.md §5.1-2 의 핵심 주장."""
    from modules.detector.geometry import object_metrics_from_points

    top_view = _cube_view_points(0.2, 0.1, -0.022, 0.023, side=None)
    side_view = _cube_view_points(0.2, 0.1, -0.022, 0.023, side="x")
    fused = np.vstack([top_view, side_view])
    m = object_metrics_from_points(fused)
    assert m is not None
    position, base_z, height = m
    assert abs(height - 0.023) < 3e-3, height  # 실 높이 복원
    assert abs(base_z - (-0.045)) < 3e-3, base_z  # 물체 바닥 = top - height
    assert abs(position[2] + 0.022) < 2e-3  # 윗면 z 유지


def test_object_metrics_zgap_cuts_below_outliers():
    """실물 #1 phantom 회귀 (§10.3-F) — mask 경계 flying-pixel/배경 누출이 물체
    **한참 아래** 점 봉우리를 만들어도 base_z 가 안 끌려간다. 옛 2-percentile
    bottom 은 outlier 3~5% 에 base_z −0.2m / height 20cm phantom (실물 로그
    일치) — z-gap 군집(top 에서 이어지는 z 덩어리만 몸통)이 수정."""
    from modules.detector.geometry import object_metrics_from_points

    body = _cube_view_points(0.2, 0.1, top_z=-0.022, height=0.023, side="x")
    rng = np.random.default_rng(0)
    n_out = max(3, int(len(body) * 0.05))  # 5% 아래-outlier
    outliers = np.stack(
        [
            0.2 + rng.normal(0, 0.02, n_out),
            0.1 + rng.normal(0, 0.02, n_out),
            rng.uniform(-0.30, -0.12, n_out),  # 물체보다 한참 아래 (phantom 영역)
        ],
        axis=1,
    )
    m = object_metrics_from_points(np.vstack([body, outliers]))
    assert m is not None
    position, base_z, height = m
    assert abs(base_z - (-0.045)) < 3e-3, base_z  # 물체 바닥 유지 — phantom 없음
    assert abs(height - 0.023) < 3e-3, height
    assert abs(position[2] + 0.022) < 2e-3


def test_object_metrics_flying_pixel_trail_above_does_not_hijack_top():
    """2026-07-17 실물 회귀 — 실루엣 flying pixel 은 카메라 쪽 = 물체 **위**로
    뜬 성긴 트레일을 만든다 (내부 간격 <5mm 로 연결, 몸통과는 >5mm 분리). 옛
    p98 top 앵커가 트레일(전체의 2% 초과)에 올라타면 z-gap 군집이 트레일만
    몸통으로 삼아 물체 전체가 공중 부양했다 — blue box base_z=+0.175 공중
    spot (resolve_place spot 당 ~55s 전멸), 큐브 top +2cm (servo 허공 목표 IK
    거부). 몸통 = **질량 있는** 최상단 군집이라야 한다 (PLY 실측: 트레일은
    수 %, 물체 면은 대다수)."""
    from modules.detector.geometry import object_metrics_from_points

    body = _cube_view_points(0.2, 0.1, top_z=0.024, height=0.023, side="x")
    n_trail = 12  # 288점 대비 ~4% — p98(하위 2% 컷)을 넘는 오염량
    trail_z = np.linspace(0.15, 0.20, n_trail)  # 간격 4.5mm — 트레일 내부 연결
    trail = np.stack(
        [np.full(n_trail, 0.2), np.full(n_trail, 0.1), trail_z], axis=1
    )
    m = object_metrics_from_points(np.vstack([body, trail]))
    assert m is not None
    position, base_z, height = m
    assert abs(position[2] - 0.024) < 2e-3, position  # top = 물체 윗면 (트레일 아님)
    assert abs(base_z - 0.001) < 3e-3, base_z  # 바닥 = top − height (공중 부양 없음)
    assert abs(height - 0.023) < 3e-3, height


def test_object_metrics_elevated_object_wins_over_larger_lower_bleed():
    """적치된(공중) 물체 — 상자 위 큐브처럼 실제로 떠 있는 물체는 base_z 가
    올라간 값으로 **정직하게** 나와야 한다 (floor 추정 없음 — 공중/손 성립이
    이 기하의 존재 이유). 동시에 mask 가 아래 면(테이블)으로 새서 만든 더 큰
    하부 군집이 있어도, 몸통 선택은 '최대 질량'이 아니라 '질량 자격을 갖춘
    최상단'이라 물체가 이긴다 — 최대-질량 선택으로 바꾸면 이 테스트가 잡는다."""
    from modules.detector.geometry import object_metrics_from_points

    # 상자(top 0.027) 위 큐브: z ∈ [0.027, 0.051] — 288점
    cube = _cube_view_points(0.2, 0.1, top_z=0.051, height=0.024, side="x")
    # 테이블(z≈0)로 샌 배경 bleed — 큐브보다 **많은** 점 (400점), 27mm gap 분리
    rng = np.random.default_rng(1)
    n_bleed = 400
    bleed = np.stack(
        [
            0.2 + rng.normal(0, 0.02, n_bleed),
            0.1 + rng.normal(0, 0.02, n_bleed),
            rng.normal(0.0, 0.001, n_bleed),
        ],
        axis=1,
    )
    m = object_metrics_from_points(np.vstack([cube, bleed]))
    assert m is not None
    position, base_z, height = m
    assert abs(position[2] - 0.051) < 2e-3, position  # top = 큐브 윗면
    assert abs(base_z - 0.027) < 3e-3, base_z  # 바닥 = 상자 top (공중 — 정직)
    assert abs(height - 0.024) < 3e-3, height


def test_voxel_downsample_reduces_and_preserves_extent():
    from modules.detector.geometry import voxel_downsample

    pts = _cube_view_points(0.0, 0.0, 0.0, 0.02, side="x", n=40)
    ds = voxel_downsample(pts, voxel_m=0.003)
    assert len(ds) < len(pts)
    # 범위 보존 (기하 재계산 소스로 유효)
    for axis in range(3):
        assert abs(ds[:, axis].min() - pts[:, axis].min()) < 0.003
        assert abs(ds[:, axis].max() - pts[:, axis].max()) < 0.003


def test_cluster_indices_by_xy_groups_same_object():
    from modules.detector.geometry import cluster_indices_by_xy

    positions = [
        (0.20, 0.10, 0.0),  # 물체 A 뷰1
        (0.21, 0.11, 0.0),  # 물체 A 뷰2 (1.4cm 옆 — 같은 물체)
        (0.35, -0.2, 0.0),  # 물체 B
    ]
    groups = cluster_indices_by_xy(positions, eps_m=0.04)
    as_sets = sorted(sorted(g) for g in groups)
    assert as_sets == [[0, 1], [2]]


def test_align_and_merge_views_corrects_systematic_view_bias():
    """멀티뷰 정합 회귀 (2026-07-14 실물 사고 그대로): 같은 큐브가 뷰마다 base
    좌표 ~3.3cm 어긋나게 관측됨 (자세별 FK bias) → naive vstack 은 25mm 큐브를
    50×64mm 얼룩으로 만들어 가짜 w=31mm antipodal 쌍(허공 파지)의 재료가 됐다.
    중심차 정렬 병합은 footprint 를 실물 크기로 유지해야 한다 (bias 는 검출
    position 에 그대로 실린다 — position = 자기 점군 centroid). 뒤집으면 깨짐."""
    from modules.detector.geometry import align_and_merge_views

    edge = 0.022
    v1 = _cube_view_points(0.259, 0.120, 0.037, 0.022, side="x", edge=edge)
    bias = np.array([-0.015, 0.029, 0.004])  # 실물 후보0↔후보1 오프셋 재현
    v2 = v1 + bias
    centers = [
        (0.259, 0.120, 0.037),
        (0.259 + float(bias[0]), 0.120 + float(bias[1]), 0.037 + float(bias[2])),
    ]
    merged = align_and_merge_views([v1, v2], centers)
    assert len(merged) == len(v1) + len(v2)  # 병합 (제외 없음)
    # 병합 후에도 실물 크기 (naive vstack 이면 y-span ≈ 22+29 = 51mm)
    assert (merged[:, 1].max() - merged[:, 1].min()) < edge + 0.008
    assert (merged[:, 0].max() - merged[:, 0].min()) < edge + 0.008


def test_align_and_merge_views_anchors_on_mean_not_single_view():
    """앵커 = 뷰 평균 (medoid 단일 뷰 아님) — 한 뷰의 bias 를 통째 물면 융합 중심이
    뷰 추가마다 휘청여 큐브 끝을 스친다 (2026-07-14 실물). 대칭 점군(centroid=center)
    으로 앵커 위치를 정확히 검증: 바깥 outlier 뷰 하나가 있어도 결과 중심은
    평균에 앉는다. 뒤집으면(medoid/한 뷰) = 중심이 그 뷰로 끌려가 깨짐."""
    from modules.detector.geometry import align_and_merge_views

    # 대칭 격자(옆면 없음) → 각 cloud 의 centroid == 넘긴 center
    def block(cx, cy):
        xs = np.linspace(-0.01, 0.01, 6)
        gx, gy = np.meshgrid(xs, xs)
        return np.stack([gx.ravel() + cx, gy.ravel() + cy, np.zeros(36)], axis=1)

    centers = [(0.20, 0.10, 0.0), (0.21, 0.11, 0.0), (0.28, 0.10, 0.0)]  # 3번째 outlier
    merged = align_and_merge_views([block(*c[:2]) for c in centers], centers)
    mean_xy = np.mean([c[:2] for c in centers], axis=0)  # (0.23, 0.1033)
    assert abs(merged[:, 0].mean() - mean_xy[0]) < 1e-6
    assert abs(merged[:, 1].mean() - mean_xy[1]) < 1e-6
