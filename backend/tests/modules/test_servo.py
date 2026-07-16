"""servo 순수 계산 테스트 — 하드웨어/wire 0 (closed-loop 의 결정적 부분 잠금).

의미 (뒤집으면 회귀): 파지 자세 가족의 tool-frame 기하(접근/조 축/lateral 방향) /
standoff 가 접근축 후방이 아님 / 오차 분해(lateral↔axial)가 접근축 기준이 아님 /
tick gate 가 mask 오검출(실데이터 455mm 사례)·점군 붕괴를 통과시킴 / decide_tick
상태 전이가 handoff §2 표(단발 hold·연속 소실 결단·보정 상한·tick 상한)와 다름 —
어느 하나가 무너지면 루프가 "감지 못 하면 크래시/무한대기" 로 돌아간다.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from modules.detector.contract import OrientedDetection
from modules.tasks.pick_and_place import servo


def _det(
    position=(0.2, 0.05, 0.025),
    base_z: float = 0.0,
    height: float = 0.025,
    grasp_yaw: float = 0.0,
    footprint=(0.025, 0.024),
    points=None,
    score: float = 0.9,
) -> OrientedDetection:
    return OrientedDetection(
        prompt="cube", position=position, score=score, base_z=base_z,
        height=height, grasp_yaw=grasp_yaw, footprint=footprint, points=points,
    )


_CFG = servo.ServoConfig()


# ─── 파지 자세 가족 ────────────────────────────────────────────────────


def test_grasp_families_count_and_preference_order():
    fams = servo.grasp_families(_det())
    # tilt 13 × 조 축 2(짧은 변 우선) × flip 2 = 52. 첫 후보 = 수직 + 짧은 변.
    assert len(fams) == 13 * 2 * 2
    assert fams[0].tilt_deg == 0 and "∥short" in fams[0].label
    assert "flip=+" in fams[0].label
    # 선호순: 작은 tilt 이 앞 (도달만 되면 수직에 가까운 쪽 채택)
    tilts = [abs(f.tilt_deg) for f in fams]
    assert tilts == sorted(tilts)


def test_grasp_family_frame_convention():
    """tool frame 규약: x=접근축, y=조 축(수평), z=x×y — tilt=0 은 수직 하강."""
    fams = servo.grasp_families(_det(grasp_yaw=0.0))
    f0 = fams[0]  # tilt=0, jaw∥short(=yaw+90°), flip=+
    assert f0.approach == pytest.approx((0.0, 0.0, -1.0))
    assert f0.jaw_axis == pytest.approx((math.cos(math.pi / 2),
                                         math.sin(math.pi / 2), 0.0), abs=1e-9)
    # quat 의 x축(접근) 이 approach 와 일치
    rot = Rotation.from_quat(f0.quat)
    assert rot.apply([1.0, 0.0, 0.0]) == pytest.approx(f0.approach, abs=1e-9)
    assert rot.apply([0.0, 1.0, 0.0]) == pytest.approx(f0.jaw_axis, abs=1e-9)


def test_grasp_family_tilt_rotates_approach_about_jaw_axis():
    fams = servo.grasp_families(_det(grasp_yaw=0.0))
    f45 = next(f for f in fams if f.tilt_deg == 45 and "∥short" in f.label
               and "flip=+" in f.label)
    # 조 축(y=+y base) 둘레 45° — 접근이 수직에서 45° 기울되 조 축은 수평 유지
    assert f45.jaw_axis == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    assert f45.approach[2] == pytest.approx(-math.cos(math.radians(45)))
    assert abs(f45.approach[2]) < 1.0  # 수직이 아님
    assert np.dot(f45.approach, f45.jaw_axis) == pytest.approx(0.0, abs=1e-9)


# ─── 폭/오프셋/파지점 ─────────────────────────────────────────────────


def test_width_along_measures_extent_on_axis():
    pts = [(0.2 + x, 0.05 + y, 0.01) for x in np.linspace(-0.02, 0.02, 20)
           for y in np.linspace(-0.01, 0.01, 10)]
    w_x = servo.width_along(pts, (1.0, 0.0, 0.0), fallback_m=0.999)
    w_y = servo.width_along(pts, (0.0, 1.0, 0.0), fallback_m=0.999)
    assert w_x == pytest.approx(0.04, abs=0.005)
    assert w_y == pytest.approx(0.02, abs=0.005)
    # 점군 없음/부족 = fallback (coarse footprint)
    assert servo.width_along(None, (1, 0, 0), fallback_m=0.123) == 0.123
    few = [(0.0, 0.0, 0.0)] * 3
    assert servo.width_along(few, (1, 0, 0), fallback_m=0.123) == 0.123


def test_grasp_point_z_from_fused_geometry_with_clamps():
    latest = _det(position=(0.21, 0.06, 0.025))
    fused = _det(position=(0.2, 0.05, 0.025), base_z=0.0, height=0.025)
    p = servo.grasp_point(latest, fused, _CFG)
    # XY = 최신 관측 (common-mode 상쇄는 최신 자세 측정에만 성립)
    assert p[0] == 0.21 and p[1] == 0.06
    # z = base_z + height/2 (25mm 큐브 → 12.5mm)
    assert p[2] == pytest.approx(0.0125)
    # height 과소(관측 부족) → 최소 파지 깊이로 방어
    shallow = servo.grasp_point(latest, _det(height=0.004), _CFG)
    assert shallow[2] == pytest.approx(_CFG.grip_depth_min_m)
    # 아주 큰 물체 → 최대 깊이 clamp (윗면 근처 파지)
    tall = servo.grasp_point(
        _det(position=(0.2, 0.05, 0.10)), _det(height=0.10), _CFG
    )
    assert tall[2] == pytest.approx(_CFG.grip_depth_max_m)


def test_grasp_tcp_applies_lateral_along_jaw_axis():
    fam = servo.grasp_families(_det(grasp_yaw=0.0))[0]  # jaw=+y
    tcp = servo.grasp_tcp((0.2, 0.05, 0.0125), fam, 0.008)
    assert tcp == pytest.approx((0.2, 0.058, 0.0125), abs=1e-9)


def test_standoff_backs_off_along_approach():
    fam = servo.grasp_families(_det())[0]  # 접근 = 수직 하강
    so = servo.standoff((0.2, 0.05, 0.0125), fam, 0.08)
    assert so == pytest.approx((0.2, 0.05, 0.0925), abs=1e-9)


def test_split_error_decomposes_about_approach():
    fam = servo.grasp_families(_det())[0]  # approach = (0,0,-1)
    lat, ax = servo.split_error((0.003, 0.004, -0.02), fam)
    assert lat == pytest.approx(0.005)  # xy 성분 (3-4-5)
    assert ax == pytest.approx(0.02)  # 접근 방향(-z) 성분


# ─── tick gate (실데이터 실패 클래스) ─────────────────────────────────


def _pts(n: int = 100) -> list:
    return [(0.2, 0.05, 0.01)] * n


def test_gate_empty_and_match_radius():
    g = servo.gate_observation([], (0.2, 0.05, 0.0), None, _CFG)
    assert g.obs is None and "검출 0건" in g.reason
    # 반경 밖 후보만 = 매치 실패 (다른 물체를 잡는 것 차단)
    far = _det(position=(0.5, 0.4, 0.02), points=_pts())
    g = servo.gate_observation([far], (0.2, 0.05, 0.0), None, _CFG)
    assert g.obs is None and "매치 실패" in g.reason
    # 반경 안 최근접 선택
    near = _det(position=(0.21, 0.05, 0.02), points=_pts())
    nearer = _det(position=(0.205, 0.052, 0.02), points=_pts())
    g = servo.gate_observation([far, near, nearer], (0.2, 0.05, 0.0), None, _CFG)
    assert g.obs is nearer


def test_gate_rejects_position_jump():
    """mask 오검출 gate — 실데이터 0003 뷰(455mm 도약) 클래스."""
    last = _det(position=(0.2, 0.05, 0.02))
    jumped = _det(position=(0.2 + 0.045, 0.05, 0.02), points=_pts())
    g = servo.gate_observation([jumped], (0.2, 0.05, 0.0), last, _CFG)
    assert g.obs is None and "도약" in g.reason


def test_gate_rejects_thin_point_cloud():
    """depth 붕괴/가림 gate — 근접 한계 신호 (실데이터 0004 뷰 valid 71.6%)."""
    thin = _det(position=(0.2, 0.05, 0.02), points=_pts(10))
    g = servo.gate_observation([thin], (0.2, 0.05, 0.0), None, _CFG)
    assert g.obs is None and "점군 부족" in g.reason


# ─── decide_tick 상태 전이 (handoff §2 표) ────────────────────────────


def _miss() -> servo.GateResult:
    return servo.GateResult(None, "검출 0건")


def _hit() -> servo.GateResult:
    return servo.GateResult(_det(points=_pts()), "")


def test_single_miss_holds_without_motion():
    st = servo.ServoState()
    d = servo.decide_tick(st, _miss(), None, _CFG)
    assert d.action == "hold" and st.misses == 1


def test_consecutive_miss_at_rung0_aborts_with_reason():
    st = servo.ServoState()
    servo.decide_tick(st, _miss(), None, _CFG)
    d = servo.decide_tick(st, _miss(), None, _CFG)
    assert d.action == "abort" and "소실" in d.reason and "rung 0" in d.reason


def test_consecutive_miss_after_convergence_commits_from_last():
    """가까이서(rung≥1) 직전 오차가 capture 안이면 마지막 관측으로 blind commit
    (그리퍼 가림/근접 한계로 못 보게 된 경우 — 후퇴가 아니라 결단)."""
    st = servo.ServoState(rung=1)
    st.last_lateral_m = 0.004
    servo.decide_tick(st, _miss(), None, _CFG)
    d = servo.decide_tick(st, _miss(), None, _CFG)
    assert d.action == "commit" and "직전 수렴 관측" in d.reason


def test_converge_descends_then_commits_at_last_rung():
    cfg = servo.ServoConfig(standoffs=(0.10, 0.05), eps_descend_m=(0.008, 0.004))
    st = servo.ServoState()
    d1 = servo.decide_tick(st, _hit(), 0.003, cfg)
    assert d1.action == "descend" and st.rung == 1
    d2 = servo.decide_tick(st, _hit(), 0.002, cfg)
    assert d2.action == "commit"


def test_above_eps_corrects_within_cap():
    st = servo.ServoState()
    d = servo.decide_tick(st, _hit(), 0.02, _CFG)
    assert d.action == "correct" and st.corrections == 1


def test_correction_cap_descends_if_within_capture_else_aborts():
    # capture 안 (12mm) — 더 가까운 측정이 더 정확하므로 하강 강행
    st = servo.ServoState()
    st.corrections = _CFG.corrections_per_rung
    d = servo.decide_tick(st, _hit(), 0.011, _CFG)
    assert d.action == "descend" and st.rung == 1
    # capture 밖 — 발진/오차 정체로 명시 중단 (사유에 오차 이력)
    st2 = servo.ServoState()
    st2.corrections = _CFG.corrections_per_rung
    st2.error_history_mm = [25.0, 24.0, 26.0]
    d2 = servo.decide_tick(st2, _hit(), 0.025, _CFG)
    assert d2.action == "abort" and "수렴 실패" in d2.reason


def test_tick_budget_aborts():
    st = servo.ServoState()
    st.ticks = _CFG.max_ticks
    d = servo.decide_tick(st, _hit(), 0.001, _CFG)
    assert d.action == "abort" and "tick 상한" in d.reason


def test_hit_resets_miss_counter():
    """단발 드롭 후 재관측 성공 — miss 카운터 리셋 (드롭 2회가 연속이 아니면
    소실로 오판하지 않는다)."""
    cfg = servo.ServoConfig(standoffs=(0.10, 0.05), eps_descend_m=(0.008, 0.004))
    st = servo.ServoState()
    servo.decide_tick(st, _miss(), None, cfg)
    servo.decide_tick(st, _hit(), 0.002, cfg)
    assert st.misses == 0
    d = servo.decide_tick(st, _miss(), None, cfg)
    assert d.action == "hold"  # 다시 단발 — 연속 아님
