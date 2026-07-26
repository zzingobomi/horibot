"""handover 계획 기하의 실-기구학 도달성 (sim) — 흉터 5(워크스페이스 전멸) 회귀.

2026-07-23 offline probe 로 잡은 구멍들을 실 URDF/캘(repo horibot.db)/base_pose
로 잠근다 (각 구멍 = 설계 변경으로 이어진 실측):
  ① omx nadir 관측 ψ 격자에 도달 자세가 있다 (ψ=90° 실측)
  ② omx top-down 파지 격자 다수 도달 (§5.1 manifold)
  ③ **제시 자세족** — omx 5축이 공중 랑데부에 도달하는 tool 자세는 접선(tool-z
     ∥ TCP 방위 접선)+spin 족뿐 (큐브 대칭이 방위를 자유롭게 한다는 착각 방지 —
     top-down/tilt 는 J2–J4 리밋 전멸). _present_orientations 가 ≥1 도달해야 한다
  ④ **so101 수취** — v1 의 toward-상대 coarse 부채꼴은 0/21 전멸 실측 → 절대
     yaw 15° 격자 × tilt 사다리가 채택 H 에서 ≥1 가족을 찾아야 한다 + 수취
     관측 사다리 ≥1
  ⑤ 채택 구성 쌍의 링크 최근접 ≥ _RECV_COLLISION_MARGIN_M (실측 11.1mm —
     margin 10mm. 노브를 흔들어 여유가 margin 밑으로 내려가면 여기서 깨진다)

노브(_PRESENT_*/_RECV_*/workcell)를 바꾸면 이 테스트가 실물 전에 먼저 비명을
지르는 것이 목적 — 실패 시 scratchpad probe 계열로 재특성화.
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
from modules.tasks.handover import frames, steps
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


def _adopt_present(env):
    """시나리오와 같은 순서로 제시 채택 — (tcp_w, sol, h_world). 일반 grasp
    자세족(_present_orientations)에서 첫 도달. H(재검출 겨냥점) = 제시 TCP =
    큐브 중심. 악수 높이(_PRESENT_Z_WORLD ~0.32)에서 성립해야 한다."""
    cands = frames.rendezvous_candidates(
        env["roi_so"], env["roi_omx"], env["base"], steps._PRESENT_Z_WORLD,
        limit=steps._PRESENT_LIMIT, prefer_r_so=steps._RENDEZVOUS_R_SO_M,
    )
    assert cands, "랑데부 교집합 비어 있음 — workcell ROI z_max(악수높이) 회귀"
    for tcp_w in cands:
        tcp_o = frames.world_to_robot(tcp_w, env["base"])
        for _label, quat in steps._present_orientations():
            sol = _ik(env["k_omx"], tcp_o, quat)
            if sol:
                return tcp_w, sol, tcp_w
    pytest.fail("제시 자세족 전멸 — _present_orientations(일반 샘플러)/악수높이 회귀")


def test_present_and_receive_feasible_with_clearance(env):
    _tcp_w, omx_sol, h = _adopt_present(env)
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

    # 수취 가족 (일반 grasp 샘플러 — 위/아래면 수직 조축 포함), 겨냥 = 큐브 중심 H.
    # ≥1 도달 + 벽밖 + 여유. 도달해가 수직 조축을 포함하는지도 확인(스윕 결론).
    tgt = h
    so_sols = []
    vert_reach = 0
    for _label, quat, a in steps._RECV_FAMILY:
        pre = tuple(tgt[i] - a[i] * steps._RECV_PRE_CLEAR_M for i in range(3))
        s_pre, s_g = _ik(env["k_so"], pre, quat), _ik(env["k_so"], tgt, quat)
        if s_pre and s_g:
            so_sols.append(s_g)
            jaw_z = abs(_R.from_quat(quat).apply([0.0, 1.0, 0.0])[2])
            if jaw_z > 0.7:
                vert_reach += 1
    assert so_sols, (
        f"so101 수취 가족 전멸 — 일반 샘플러(_RECV_FAMILY)/악수높이 회귀 (H={h})"
    )
    assert vert_reach, "수취 도달해에 수직 조축(위/아래면)이 없음 — 스윕 결론 회귀"

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
