"""표면 antipodal 파지 선택 테스트 (순수 계산 — open3d 법선, 하드웨어 0).

⚠ antipodal/plan_grasp 는 production 소비자 없음 (2026-07-16 closed-loop 전환 —
servo.grasp_families 가 대체). scripts/grasp_verify/ 진단 자산(post-mortem §9)과
test_motion 의 resolve 게이트 sim 이 소비해 수식을 계속 잠근다.

의미 (뒤집으면 회귀): 마주 보는 두 면이 관측돼야 쌍이 나온다(단일 뷰 0쌍) /
조 개구보다 넓은 물체는 못 문다 / 조 축 수평 필터(수평 마주면만) / plan_grasp
후보의 tool-frame 기하 (접근축 후퇴 pre, 단일 조 lateral, tilt 우선순위).
"""

from __future__ import annotations


import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from modules.tasks.pick_and_place import geometry
from modules.tasks.pick_and_place.antipodal import (
    AntipodalPair,
    horizontal_antipodal_pairs,
)


def _face_x(
    x: float, cy: float, cz: float, *, width: float, height: float, n: int = 14
) -> np.ndarray:
    """x=const 수직 면 (법선 ±x) — y/z 격자."""
    ys = np.linspace(cy - width / 2, cy + width / 2, n)
    zs = np.linspace(cz - height / 2, cz + height / 2, n)
    gy, gz = np.meshgrid(ys, zs)
    return np.stack([np.full(n * n, x), gy.ravel(), gz.ravel()], axis=1)


def _face_z(
    z: float, cx: float, cy: float, *, wx: float, wy: float, n: int = 14
) -> np.ndarray:
    """z=const 수평 면 (법선 ±z) — x/y 격자."""
    xs = np.linspace(cx - wx / 2, cx + wx / 2, n)
    ys = np.linspace(cy - wy / 2, cy + wy / 2, n)
    gx, gy = np.meshgrid(xs, ys)
    return np.stack([gx.ravel(), gy.ravel(), np.full(n * n, z)], axis=1)


def _box_cloud(
    cx: float = 0.20, cy: float = 0.05, *, gap: float = 0.022,
    both_sides: bool = True,
) -> np.ndarray:
    """수직 옆면 2개(±x, 간격 gap) + 윗면 — 멀티뷰 융합 점군 흉내.

    both_sides=False = 단일 뷰 (마주 보는 면 중 먼 쪽이 가려짐 — §10.3-B).
    """
    cz, h = -0.034, 0.023  # 윗면 -0.0225, 바닥 -0.0455 근방
    faces = [_face_x(cx + gap / 2, cy, cz, width=0.022, height=h)]
    if both_sides:
        faces.append(_face_x(cx - gap / 2, cy, cz, width=0.022, height=h))
    faces.append(_face_z(cz + h / 2, cx, cy, wx=gap, wy=0.022))
    return np.vstack(faces)


def test_pairs_require_both_sides_single_view_zero():
    """단일 뷰(한쪽 옆면만) = 0쌍 — 마주 보는 면이 없다. 멀티뷰 융합(양쪽) =
    쌍 생성 + 폭/조 축이 실제 기하와 일치. §10.3-B 그대로."""
    single = horizontal_antipodal_pairs(_box_cloud(both_sides=False))
    assert single == []

    pairs = horizontal_antipodal_pairs(_box_cloud(both_sides=True))
    assert pairs, "양쪽 옆면 관측인데 antipodal 쌍 0"
    for p in pairs:
        assert abs(p.width - 0.022) < 0.006, p  # 접촉 폭 ≈ 면 간격
        assert abs(p.jaw_axis[2]) < 1e-9  # 조 축 수평 (z 성분 0)
        assert abs(abs(p.jaw_axis[0]) - 1.0) < 0.1, p  # 조 축 ≈ ±x (면 법선 방향)


def test_pairs_reject_wider_than_jaw_open():
    """조 최대 개구(3.5cm)보다 넓은 물체는 쌍이 안 나온다 — 못 무는 파지를
    후보로 만들면 실행 게이트 비용만 태운다."""
    assert horizontal_antipodal_pairs(_box_cloud(gap=0.05)) == []


def test_pairs_horizontal_only():
    """수평 마주면(윗면/아랫면 — 법선 수직)만 있는 관측 = 0쌍 — SO-101 옆파지는
    조 축 수평이어야 성립 (top/bottom 을 무는 자세는 물리적으로 없음)."""
    top = _face_z(-0.0225, 0.20, 0.05, wx=0.022, wy=0.022)
    bottom = _face_z(-0.0455, 0.20, 0.05, wx=0.022, wy=0.022)
    assert horizontal_antipodal_pairs(np.vstack([top, bottom])) == []


def test_pairs_too_few_points():
    assert horizontal_antipodal_pairs(np.zeros((5, 3))) == []


# ─── plan_grasp — 접촉쌍 → 접근 후보 가족 ────────────────────────────


def _pair(width: float = 0.022) -> AntipodalPair:
    return AntipodalPair(mid=(0.20, 0.05, -0.034), jaw_axis=(0.0, 1.0, 0.0),
                         width=width)


def test_plan_grasp_family_counts_and_tilt_order():
    plan = geometry.plan_grasp([_pair(), _pair(width=0.015)])
    assert len(plan) == 13 * 2 * 2  # tilt(0~±90) × 쌍 2 × flip
    assert "tilt=+0" in plan[0].label  # 작은 tilt 우선 (도달만 되면 수직 선호)


def test_plan_grasp_tool_frame_geometry():
    """tilt=0 후보의 tool-frame 기하 — 접근축 수직 하향, TCP 가 mid 에서 조 축
    방향 lateral(단일 가동 조 보정)만큼 이동, pre 는 접근축 후방 0.06m."""
    c = geometry.plan_grasp([_pair()])[0]
    lateral = 0.022 / 2 + 0.005 - 0.0079
    assert c.lateral == pytest.approx(lateral)
    # tilt=0: 접근축 = -z → grasp = mid + (0, lateral, 0), pre = grasp + (0,0,0.06)
    assert c.grasp == pytest.approx((0.20, 0.05 + lateral, -0.034))
    assert c.pre == pytest.approx((0.20, 0.05 + lateral, -0.034 + 0.06))
    rot = Rotation.from_quat(c.quat).as_matrix()
    assert rot[:, 0] == pytest.approx([0.0, 0.0, -1.0])  # x_tool = 접근축
    assert rot[:, 1] == pytest.approx([0.0, 1.0, 0.0])  # y_tool = 조 축


def test_plan_grasp_pre_is_along_approach_axis():
    """모든 후보: |pre−grasp| = 접근 거리 (접근축 직선 진입 전제) + 기울인
    후보의 pre 는 수직 위가 아니다 (grasp-frame 동작 §5.4 회귀 잠금)."""
    plan = geometry.plan_grasp([_pair()])
    for c in plan:
        gap = np.array(c.pre) - np.array(c.grasp)
        assert np.linalg.norm(gap) == pytest.approx(0.06, abs=1e-9), c.label
    tilted = [c for c in plan if "tilt=+60" in c.label]
    assert tilted and any(
        abs(c.pre[0] - c.grasp[0]) > 0.01 or abs(c.pre[1] - c.grasp[1]) > 0.01
        for c in tilted
    )


def test_plan_grasp_flip_mirrors_lateral():
    """flip = 조 축 반전 — 단일 가동 조 lateral 이 반대쪽으로 걸린다 (두 후보가
    서로 다른 도달성/충돌 프로파일을 갖는 이유)."""
    plan = geometry.plan_grasp([_pair()])
    t0 = [c for c in plan if "tilt=+0" in c.label]
    assert len(t0) == 2
    plus, minus = t0[0], t0[1]
    assert plus.grasp[1] == pytest.approx(0.05 + plus.lateral)
    assert minus.grasp[1] == pytest.approx(0.05 - minus.lateral)


def test_grasp_ik_groups_pair_pre_and_grasp():
    plan = geometry.plan_grasp([_pair()])
    groups = geometry.grasp_ik_groups(plan)
    assert len(groups) == len(plan)
    assert groups[0][0].position == plan[0].pre
    assert groups[0][1].position == plan[0].grasp
    assert groups[0][0].quaternion == plan[0].quat


# (옛 adaptive 뷰 탐색축 view_directions/view_pose_groups 테스트는 2026-07-16
# closed-loop 전환으로 함수와 함께 삭제 — 뷰 이동이 servo 루프로 대체됨.)
