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


def test_grasp_point_z_anchored_to_top_surface():
    latest = _det(position=(0.21, 0.06, 0.025))
    fused = _det(position=(0.2, 0.05, 0.025), base_z=0.0, height=0.025)
    p = servo.grasp_point(latest, fused, _CFG)
    # XY = 최신 관측 (common-mode 상쇄는 최신 자세 측정에만 성립)
    assert p[0] == 0.21 and p[1] == 0.06
    # z = 윗면 − grip_below_top — base_z 앵커 아님 (단일 top-view 의 base_z 는
    # ≈윗면이라 nip 튕김 실사고, 2026-07-16)
    assert p[2] == pytest.approx(0.025 - _CFG.grip_below_top_m)
    # 단일 뷰 band height(4mm) 는 신뢰 밖 — 깊이가 band 두께에 안 끌려간다
    shallow = servo.grasp_point(latest, _det(height=0.004), _CFG)
    assert shallow[2] == pytest.approx(0.025 - _CFG.grip_below_top_m)
    # 신뢰 가능한 height + 깊은 grip 설정 → "관측 바닥 +4mm" 하한이 지킨다
    deep = servo.ServoConfig(grip_below_top_m=0.020)
    thin = servo.grasp_point(latest, _det(height=0.016), deep)
    assert thin[2] == pytest.approx(0.025 - 0.016 + 0.004)


def test_grasp_point_floor_clamp_for_flat_objects():
    """납작한 물체(height < credible → 바닥 guard 비활성)에서 grip_below_top 이
    테이블을 뚫지 않게 — floor_z(실 바닥 추정)가 마지막 하한 (모양 가정 없이
    plan 관측 데이터만)."""
    flat_top = 0.006  # 높이 ~8mm 물체의 윗면 (바닥 -0.002)
    latest = _det(position=(0.2, 0.05, flat_top), height=0.006)
    # floor 없음 → top−10mm = 테이블 아래 (구멍이었던 동작)
    no_guard = servo.grasp_point(latest, latest, _CFG, None)
    assert no_guard[2] < -0.002
    # floor 지정 → 바닥 +floor_clear 위로 clamp (상한 top−2mm 안에서)
    guarded = servo.grasp_point(latest, latest, _CFG, -0.002)
    assert guarded[2] == pytest.approx(-0.002 + _CFG.floor_clear_m)
    assert guarded[2] <= flat_top - 0.002


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


def test_plant_comp_feedforward_and_clamp():
    """PlantComp — 실행된 명령만 기준으로 잔차 학습, 상수 오프셋 1스텝 소거,
    clamp 폭주 방지 (2026-07-16 lateral 정체 사고의 해법 잠금)."""
    comp = servo.PlantComp(max_m=0.03)
    # 명령 이력 없으면 보상 0 (첫 tick)
    comp.observe((0.2, 0.1, 0.05))
    assert comp.apply((0.2, 0.1, 0.05)) == (0.2, 0.1, 0.05)
    # 명령 (0.2,0.1,0.05) → 실측이 (0.19,0.1,0.04) = 플랜트가 (10,0,10)mm 미달
    comp.commanded((0.2, 0.1, 0.05))
    comp.observe((0.19, 0.1, 0.04))
    cmd = comp.apply((0.2, 0.1, 0.05))
    assert cmd == pytest.approx((0.21, 0.1, 0.06))  # 미달만큼 선보상
    # clamp — 60mm 이상(異常) 잔차도 ±30mm 로 제한
    comp.commanded((0.2, 0.1, 0.05))
    comp.observe((0.14, 0.1, 0.05))
    assert comp.apply((0.0, 0.0, 0.0))[0] == pytest.approx(0.03)


def test_refit_family_follows_object_rotation():
    """재획득 시 물체가 회전했으면 같은 변형의 가족을 새 yaw 로 재유도 —
    옛 각도 스큐 close 재튕김 실사고 (2026-07-17 test4: 86°→-27°)."""
    fam0 = servo.grasp_families(_det(grasp_yaw=math.radians(86.0)))[0]
    # 24° 회전 (mod 90) → 재유도
    rotated = _det(grasp_yaw=math.radians(-27.5))
    refit = servo.refit_family(fam0, rotated)
    assert refit is not None and refit.label == fam0.label
    a = math.degrees(math.atan2(refit.jaw_axis[1], refit.jaw_axis[0]))
    a0 = math.degrees(math.atan2(fam0.jaw_axis[1], fam0.jaw_axis[0]))
    assert abs((a - a0 + 90.0) % 180.0 - 90.0) > 10.0
    # 소폭(<10°) 차이는 유지 (자세 고정 계약 — 마구 돌리지 않는다)
    near = _det(grasp_yaw=math.radians(82.0))
    assert servo.refit_family(fam0, near) is None


def test_gate_rejects_top_z_jump():
    """윗면 z 도약 gate — top-앵커 파지 z 의 안전망 (2026-07-17 실물: 조/가림
    depth 오염으로 top +2cm 점프 → 파지 목표 허공 → 이동 IK 거부 중단)."""
    last = _det(position=(0.2, 0.05, 0.025))
    poisoned = _det(position=(0.2, 0.05, 0.046), points=_pts())  # +21mm
    g = servo.gate_observation([poisoned], (0.2, 0.05, 0.0), last, _CFG)
    assert g.obs is None and "z 도약" in g.reason


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
