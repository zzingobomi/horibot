"""handover 계획 기하의 실-기구학 도달성 (sim) — 흉터 5(워크스페이스 전멸) 회귀.

offline probe 로 잡은 구멍들을 실 URDF/캘(repo horibot.db)/base_pose 로 잠근다
(각 구멍 = 설계 변경으로 이어진 실측. 2026-07-27 봉(8×2×2)/B-down 전환판 —
probe = scripts/handover_block_probe.py):
  ① omx nadir 관측 ψ 격자에 도달 자세가 있다 (ψ=90° 실측)
  ② omx top-down 파지 격자 다수 도달 (§5.1 manifold)
  ③ **수평(omx 접선) 제시** — 랑데부(prefer = 2026-07-28 토크오프 실측 TCP,
     z 0.18~0.22)에서 omx 접선족 × jaw-수직 quat(다양체 위 구성)이 수취 결합
     게이트까지 포함해 ≥1 채택돼야 하고, **hang 폴백이 아니어야** 한다
     (2026-07-29 회귀 — 접선을 so101 기준으로 만들면 다양체 밖 → 수평 전멸).
  ④ **so101 수취** — 봉 축 정렬족(_recv_orients, tool z ∥ w)이
     E(= 제시 TCP + w·tcp_to_e)에서 [pre, grasp] ≥1 도달 + 수취 관측 사다리 ≥1
  ⑤ 채택 구성 쌍의 링크 최근접 ≥ _RECV_COLLISION_MARGIN_M (봉 축 방향 두
     그리퍼 이격 ~2.5cm — 노브를 흔들어 여유가 margin 밑으로 내려가면 깨진다)

노브(_PRESENT_*/_RECV_*/봉 기하/workcell)를 바꾸면 이 테스트가 실물 전에 먼저
비명을 지르는 것이 목적 — 실패 시 scripts/handover_block_probe.py 로 재특성화.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as _R

from apps.config import _ROBOT_DIR, load_robots
from infra.database.boot import open_database
from modules.calibration.persistence.repository import CalibrationRepository
from modules.motion.adapters.pybullet import PybulletKinematics
from modules.motion.kinematics_builder import build_calibrated_kinematics
from modules.motor.contract import MotorKind
from modules.shared_config.contract import WorkcellRoi
from modules.tasks.handover import block, frames, steps
from modules.tasks.handover.collision import BasePose, CrossRobotChecker

pytestmark = pytest.mark.sim  # PyBullet/URDF/DB 부팅 — fast loop 제외

_DB = "sqlite:///./horibot.db"  # repo 루트 DB (git tracked — 캘 SSOT)


@pytest.fixture(scope="module")
def env():
    robots = load_robots()
    so, omx = robots["so101_6dof_0"], robots["omx_f_0"]
    if so.workcell is None or omx.workcell is None:
        pytest.skip("instance.yaml workcell 미설정")
    base = BasePose(
        omx.base_pose.x, omx.base_pose.y, omx.base_pose.z,
        math.radians(omx.base_pose.yaw_deg),
    )
    _engine, sf = open_database(_DB)
    repo = CalibrationRepository(sf)
    b_so = repo.get_active_bundle("so101_6dof_0")
    b_omx = repo.get_active_bundle("omx_f_0")
    if b_so.hand_eye is None or b_omx.hand_eye is None:
        pytest.skip("hand_eye 캘 없음 (DB)")
    arm_so = [m for m in so.motors if m.kind != MotorKind.GRIPPER]
    k_so = build_calibrated_kinematics(
        _ROBOT_DIR / so.type / "urdf" / f"{so.type}.urdf",
        "so101_6dof_0", arm_so, b_so, PybulletKinematics,
    ).kinematics
    k_so.initialize()
    k_omx = PybulletKinematics(
        _ROBOT_DIR / omx.type / "urdf" / f"{omx.type}.urdf"
    )
    k_omx.initialize()

    def he(bundle):
        x = np.eye(4)
        x[:3, :3] = np.array(bundle.hand_eye.result_data.R_cam2gripper, float)
        x[:3, 3] = np.array(
            bundle.hand_eye.result_data.t_cam2gripper, float
        ).reshape(3)
        return x

    def roi(rc):
        return WorkcellRoi(**{f: getattr(rc.workcell, f) for f in (
            "x_min", "x_max", "y_min", "y_max", "z_min", "z_max")})

    yield {
        "so": so, "omx": omx, "base": base,
        "k_so": k_so, "k_omx": k_omx,
        "x_so": he(b_so), "x_omx": he(b_omx),
        "roi_so": roi(so), "roi_omx": roi(omx),
    }
    k_so.close()
    k_omx.close()


def _ik(k, pos, quat):
    q = tuple(quat) if quat is not None else None
    return k.ik(tuple(pos), q, current_joint_angles=[0.0] * k.dof)


def test_omx_observe_pose_reachable(env):
    roi = env["roi_omx"]
    look = ((roi.x_min + roi.x_max) / 2, (roi.y_min + roi.y_max) / 2)
    c = np.array([
        look[0], look[1], steps._OMX_TABLE_Z_M + steps._OMX_OBSERVE_CAM_H_M
    ])
    groups, _m = steps._camera_pose_groups(
        c, np.array([0.0, 0.0, -1.0]), steps._OMX_OBSERVE_PSI_DEG, env["x_omx"]
    )
    ok = [g for g in groups if _ik(env["k_omx"], g[0].position, g[0].quaternion)]
    assert ok, "omx nadir 관측 ψ 격자 전멸 — 카메라 높이/hand_eye/ψ 격자 회귀"


def test_omx_topdown_pick_grid(env):
    ok = tried = 0
    for r in (0.16, 0.20, 0.24, 0.28):
        for az_deg in (-30, 0, 30):
            az = math.radians(az_deg)
            pos = (r * math.cos(az), r * math.sin(az), 0.008)
            for roll in range(0, 180, 45):
                tried += 1
                if _ik(env["k_omx"], pos,
                       steps._grasp_quat(az + math.radians(roll), 0)):
                    ok += 1
    assert ok >= tried * 0.6, f"omx top-down 격자 {ok}/{tried} — §5.1 회귀"


# 봉 명목 기하 — steps 노브에서 유도 (시나리오 plan_block_grasp_from 과 동일 식)
_BLOCK_GEOM = block.plan_block_grasp(
    (0.20, 0.0), 0.0, (0.080, 0.020),
    grasp_frac=steps._BLOCK_GRASP_FRAC,
    jaw_along_m=steps._OMX_JAW_ALONG_M,
    exposed_frac=steps._BLOCK_EXPOSED_FRAC,
    min_exposed_m=steps._SO_MIN_GRASP_M + steps._EXPOSED_MARGIN_M,
    len_min_m=steps._BLOCK_LEN_MIN_M,
    len_max_m=steps._BLOCK_LEN_MAX_M,
)


def _receive_exists(env, chk, e, w, omx_sol) -> bool:
    """steps._receive_probe 의 sim 동형 — spin 사다리 앞 6개 × 접근 여유
    사다리에서 [pre, grasp] IK + 벽 + 충돌여유 통과 해가 있는가."""
    e3 = (float(e[0]), float(e[1]), float(e[2]))
    w3 = (float(w[0]), float(w[1]), float(w[2]))
    for _label, quat, a in steps._recv_orients(e3, w3)[:6]:
        for clear_m in steps._RECV_PRE_CLEAR_LADDER:
            pre = tuple(e[i] - a[i] * clear_m for i in range(3))
            if _ik(env["k_so"], pre, quat) is None:
                continue
            s_g = _ik(env["k_so"], tuple(e), quat)
            if s_g is None:
                continue
            if chk.min_link_world_x("a", s_g, grip=1.0) < steps._WALL_MIN_X_M:
                continue
            if chk.in_collision(
                s_g, omx_sol, grip_a=1.0,
                grip_b=steps._OMX_HOLD_GRIP_FRAC,
                margin_m=steps._RECV_COLLISION_MARGIN_M,
            ):
                continue
            return True
    return False


def _adopt_present(env, chk):
    """시나리오와 같은 순서로 제시 채택 — (tcp_w, sol, h_world, w, label).

    축 일반형 (2026-07-28) + 수취 결합 게이트 (2026-07-29): 랑데부 TCP ×
    **노출 방향 w 후보**(omx 접선 × elevation + hang 폴백)를 선호순으로 돌며
    도달·손목·E-ROI·**수취 probe** 첫 통과 조합을 채택한다 — plan_omx_present
    와 동형. E = TCP + w·tcp_to_e (봉 축 위).

    ⚠ 2026-07-28 수평 제시 전환 근거: 매달기는 omx 그리퍼가 봉 위에 있어 so101
    관측 시야를 구조적으로 막았다 (실물 관측 128자세 전부 가림).
    ⚠ 2026-07-29 근인 회귀: 접선은 **omx base 기준** — so101 기준 접선은 이
    직각 배치에서 다양체 밖이라 수평 전원이 자세 IK 전멸했다."""
    cands = frames.rendezvous_candidates(
        env["roi_so"], env["roi_omx"], env["base"], steps._PRESENT_Z_WORLD,
        limit=steps._PRESENT_LIMIT, prefer_point=steps._RENDEZVOUS_PREFER_XY,
    )
    assert cands, "랑데부 교집합 비어 있음 — workcell ROI z_max 회귀"
    roi_so = env["roi_so"]
    # 수평 전 소진 후 hang — plan_omx_present 의 2-pass 순서와 동형 (앞 랑데부
    # hang 이 뒤 랑데부 수평을 가리는 순서 버그 회귀 방지)
    attempts = [
        (tcp_w, label, w)
        for hang_pass in (False, True)
        for tcp_w in cands
        for label, w in steps._present_w_candidates(tcp_w, env["base"])
        if (label == "hang") == hang_pass
    ]
    for tcp_w, label, w in attempts:
        tcp_o = frames.world_to_robot(tcp_w, env["base"])
        alpha = math.atan2(tcp_o[1], tcp_o[0])
        e = (
            tcp_w[0] + w[0] * _BLOCK_GEOM.tcp_to_e_m,
            tcp_w[1] + w[1] * _BLOCK_GEOM.tcp_to_e_m,
            tcp_w[2] + w[2] * _BLOCK_GEOM.tcp_to_e_m,
        )
        if not (
            roi_so.x_min <= e[0] <= roi_so.x_max
            and roi_so.y_min <= e[1] <= roi_so.y_max
            and roi_so.z_min <= e[2] <= roi_so.z_max
        ):
            continue  # 시나리오 E-ROI 게이트와 동형
        w_omx = frames.world_dir_to_robot(w, env["base"])
        quat = steps._present_quat_axis(w_omx, alpha)
        sol = _ik(env["k_omx"], tcp_o, quat)
        # 손목 뒤집힘 기각 — plan_omx_present 와 동형 (케이블 안전 불변식)
        if not (sol and abs(sol[-1]) <= steps._WRIST_NATURAL_MAX_RAD):
            continue
        if not _receive_exists(env, chk, e, w, sol):
            continue  # 수취 결합 게이트와 동형
        return tcp_w, sol, e, w, label
    pytest.fail("제시(+수취 결합) 전멸 — _present_w_candidates/랑데부 밴드 회귀 "
                "(scripts/handover_layout_tune.py 로 재특성화)")


def test_present_and_receive_feasible_with_clearance(env):
    so_t, omx_t = env["so"].type, env["omx"].type
    chk_adopt = CrossRobotChecker(
        _ROBOT_DIR / so_t / "urdf" / f"{so_t}.urdf",
        _ROBOT_DIR / omx_t / "urdf" / f"{omx_t}.urdf", env["base"],
    )
    try:
        _tcp_w, omx_sol, h, w, label = _adopt_present(env, chk_adopt)
    finally:
        chk_adopt.close()
    # 2026-07-29 회귀 — 수평(접선) 제시가 채택돼야 한다. hang 폴백이 뜨면
    # 접선 frame/다양체 회귀 (hang 은 관측 시선이 구조적으로 막혀 실물 전멸).
    assert label != "hang", "수평 제시 전멸 → hang 폴백 — 접선 frame 회귀"
    # so101 수취 관측 사다리 ≥1
    az0 = math.atan2(h[1], h[0])
    obs_ok = False
    for az_off in steps._RECV_OBS_AZOFF_DEG:
        for elev_deg in steps._RECV_OBS_ELEV_DEG:
            for dist in steps._RECV_OBS_DIST_M:
                az = az0 + math.radians(az_off)
                elev = math.radians(elev_deg)
                c = np.array([
                    h[0] - math.cos(az) * dist * math.cos(elev),
                    h[1] - math.sin(az) * dist * math.cos(elev),
                    h[2] + dist * math.sin(elev),
                ])
                g, _m = steps._camera_pose_groups(
                    c, np.asarray(h, float) - c,
                    steps._RECV_OBS_PSI_DEG, env["x_so"],
                )
                if any(
                    _ik(env["k_so"], gg[0].position, gg[0].quaternion)
                    for gg in g
                ):
                    obs_ok = True
                    break
            if obs_ok:
                break
        if obs_ok:
            break
    assert obs_ok, "so101 수취 관측 사다리 전멸 — _RECV_OBS_* 회귀"

    # 수취 가족 (tool z ∥ **봉 축 w**), 겨냥 = E (노출부). ≥1 [pre, grasp] 도달.
    # 자세족이 실제로 봉 축에 정렬됨도 잠근다 (축 하드코딩 회귀 방지).
    tgt = h
    w_unit = np.asarray(w, dtype=float)
    w_unit = w_unit / np.linalg.norm(w_unit)
    so_sols = []
    for _label, quat, a in steps._recv_orients(tgt, w):
        tool_z = _R.from_quat(quat).apply([0.0, 0.0, 1.0])
        assert abs(float(np.dot(tool_z, w_unit))) > 0.99, (
            "수취 자세족 tool z 가 봉 축에 정렬되지 않음 — 설계 회귀"
        )
        for clear_m in steps._RECV_PRE_CLEAR_LADDER:  # 접근 여유 사다리
            pre = tuple(tgt[i] - a[i] * clear_m for i in range(3))
            s_pre, s_g = _ik(env["k_so"], pre, quat), _ik(env["k_so"], tgt, quat)
            if s_pre and s_g:
                so_sols.append(s_g)
                break
    assert so_sols, (
        f"so101 수취 가족 전멸 — _recv_orients/E 위치 회귀 (E={h}, w={w})"
    )

    so_t, omx_t = env["so"].type, env["omx"].type
    chk = CrossRobotChecker(
        _ROBOT_DIR / so_t / "urdf" / f"{so_t}.urdf",
        _ROBOT_DIR / omx_t / "urdf" / f"{omx_t}.urdf", env["base"],
    )
    try:
        # 벽밖 + 충돌여유(margin) 통과 가족 ≥1 (실행 alive-loop 과 동형). 악수
        # 높이라 여유가 σ_t 위(스윕 16~22mm)여야 정상 — 전멸이면 높이/랑데부 회귀.
        clear = [
            s for s in so_sols
            if chk.min_link_world_x("a", s, grip=1.0) >= steps._WALL_MIN_X_M
            and not chk.in_collision(
                s, omx_sol, grip_a=1.0,
                grip_b=steps._OMX_HOLD_GRIP_FRAC,
                margin_m=steps._RECV_COLLISION_MARGIN_M,
            )
        ]
        assert clear, (
            f"수취 가족 {len(so_sols)}개 전부 벽/여유 미달 (margin "
            f"{steps._RECV_COLLISION_MARGIN_M * 1000:.0f}mm) — handoff_sweep 재실행 "
            "으로 재특성화 (악수 높이면 여유 16~22mm 나와야 정상)"
        )
    finally:
        chk.close()
