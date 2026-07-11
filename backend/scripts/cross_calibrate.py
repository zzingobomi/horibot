"""Cross-calibration — 두 robot 의 상대 base_pose 산출 (공유 보드 방식).

원리: hand-eye 캘 완료 robot 은 보드 1회 관측으로 T_base←board 를 안다:
    T_base←board = FK(q) · T_ee←cam(hand_eye) · T_cam←board(PnP)
같은 보드(두 세션 사이 이동 금지!)를 두 robot 이 각자 N회 관측 → robust 평균 →
    T_Abase←Bbase = T_Abase←board · (T_Bbase←board)⁻¹
robot A(anchor) 를 원점에 두고 B 의 base_pose 를 (x, y, z, yaw) 로 투영한다.
같은 테이블 위 robot 이라 roll/pitch ≈ 0 이 sanity check.

입력 = kind="cross" capture run (UI 캘 패널에서 robot 별 6~8장). 캡처에
board_in_cam 이 캐시돼 있어 blob 불필요 — DB 만 읽는다. 각 robot 의 active
bundle(hand_eye 필수, joint_offset/link_offset/sag 있으면 적용)로 FK 보정.

출력 = 리포트 + robots.yaml 에 붙여넣을 base_pose 블록 (yaml 자동 수정 안 함 —
placement SSOT 는 사람이 확인 후 커밋).

backend(runtime) 떠 있으면 RDB lock 충돌 — 종료 후 실행 권장.

CLI:
  uv run python scripts/cross_calibrate.py --db horibot.db
  ... --robot-a so101_6dof_0 --robot-b omx_f_0 --run-a 5 --run-b 6
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

# Repo imports (script standalone) — backend 를 path 에.
BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))

from apps.config import _ROBOT_DIR, RobotConfig, load_robots  # noqa: E402
from infra.database.sqlite import open_sqlite  # noqa: E402
from modules.calibration.contract import (  # noqa: E402
    HandEyeResultRecord,
    JointOffsetResultRecord,
    LinkOffsetResultRecord,
    SagOffsetResultRecord,
)
from modules.calibration.persistence.repository import (  # noqa: E402
    CalibrationRepository,
)
from modules.motion.fk_chain import FkChain  # noqa: E402
from modules.motion.units import raw_to_rad  # noqa: E402
from modules.motor.contract import MotorKind  # noqa: E402
from scripts.calibrate_offline import average_se3, fk_with_sag  # noqa: E402

logger = logging.getLogger(__name__)

# 같은 테이블 가정의 sanity 임계 — roll/pitch 잔차가 이보다 크면 캘/보드 의심.
PLANAR_TILT_WARN_DEG = 2.0
PLANAR_TILT_RED_DEG = 5.0
# per-robot T_base←board 흩어짐 경고 임계 (per-robot effective σ 수준).
SCATTER_ROT_WARN_DEG = 1.5
SCATTER_T_WARN_MM = 15.0


# ─── 순수 수학 (unit test 대상 — I/O 없음) ─────────────────────────


@dataclass
class ObservationStats:
    """한 robot 의 보드 관측 평균 + 흩어짐 (자기 캘 consistency 지표)."""

    T_base_board: np.ndarray  # (4, 4) 평균
    per_obs_rot_deg: list[float]  # 평균 대비 회전차
    per_obs_t_mm: list[float]  # 평균 대비 거리차


@dataclass
class PlanarPose:
    """base_pose (robots.yaml 스키마) + 투영 잔차."""

    x: float
    y: float
    z: float
    yaw_deg: float
    roll_deg: float  # 잔차 — 같은 테이블이면 ≈0
    pitch_deg: float  # 잔차 — 같은 테이블이면 ≈0


def average_board_observations(Ts: list[np.ndarray]) -> ObservationStats:
    """T_base←board 관측 N개 → robust 평균 + per-obs 흩어짐."""
    if not Ts:
        raise ValueError("관측 0개")
    R_mean, t_mean = average_se3([T[:3, :3] for T in Ts], [T[:3, 3] for T in Ts])
    T_mean = np.eye(4)
    T_mean[:3, :3] = R_mean
    T_mean[:3, 3] = t_mean
    rot_devs, t_devs = [], []
    for T in Ts:
        R_rel = R_mean.T @ T[:3, :3]
        ang = float(np.degrees(np.linalg.norm(Rot.from_matrix(R_rel).as_rotvec())))
        rot_devs.append(ang)
        t_devs.append(float(np.linalg.norm(T[:3, 3] - t_mean) * 1000.0))
    return ObservationStats(T_mean, rot_devs, t_devs)


def compose_a_from_b(T_a_board: np.ndarray, T_b_board: np.ndarray) -> np.ndarray:
    """T_Abase←Bbase = T_Abase←board · (T_Bbase←board)⁻¹."""
    return T_a_board @ np.linalg.inv(T_b_board)


def project_planar(T: np.ndarray) -> PlanarPose:
    """SE(3) → base_pose (x, y, z, yaw) + roll/pitch 잔차 (ZYX euler)."""
    yaw, pitch, roll = Rot.from_matrix(T[:3, :3]).as_euler("ZYX", degrees=True)
    return PlanarPose(
        x=float(T[0, 3]),
        y=float(T[1, 3]),
        z=float(T[2, 3]),
        yaw_deg=float(yaw),
        roll_deg=float(roll),
        pitch_deg=float(pitch),
    )


# ─── robot 별 관측 로드 + FK 합성 ──────────────────────────────────


def load_base_board_observations(
    repo: CalibrationRepository,
    robot: RobotConfig,
    run_id: int | None,
    drop_poses: set[int] | None = None,
) -> tuple[int, list[np.ndarray]]:
    """robot 의 cross run 캡처 → T_base←board 리스트.

    active bundle 적용 순서 = Motion consumer 와 동형 (joint_offset 가산,
    link_offset 은 FkChain fk 변수, sag 는 fk_with_sag).
    """
    robot_id = robot.id
    if run_id is None:
        candidates = [
            r
            for r in repo.list_runs(robot_id, "cross")
            if r.status == "ready_for_analysis"
        ]
        if not candidates:
            raise RuntimeError(
                f"cross run 없음 (robot={robot_id} — UI 캘 패널에서 cross 세션 "
                "캡처 + finalize 먼저)"
            )
        run_id = candidates[0].id  # list_runs id desc → 최신
        assert run_id is not None
    run = repo.get_run(run_id)
    if run is None or run.robot_id != robot_id:
        raise RuntimeError(f"run {run_id} 없음 또는 robot 불일치 ({robot_id})")
    if run.kind != "cross":
        raise RuntimeError(f"run {run_id} kind={run.kind!r} — cross run 아님")

    # active bundle
    he = repo.get_active(robot_id, "hand_eye")
    if not isinstance(he, HandEyeResultRecord):
        raise RuntimeError(f"active hand_eye 없음 (robot={robot_id}) — 크로스캘 불가")
    T_ee_cam = np.eye(4)
    T_ee_cam[:3, :3] = np.array(he.result_data.R_cam2gripper, dtype=np.float64)
    T_ee_cam[:3, 3] = np.array(
        he.result_data.t_cam2gripper, dtype=np.float64
    ).reshape(3)

    arm_specs = [m for m in robot.motors if m.kind != MotorKind.GRIPPER]
    spec_by_id = {m.id: m for m in arm_specs}
    n_arm = len(arm_specs)

    joint_off = np.zeros(n_arm)
    jo = repo.get_active(robot_id, "joint_offset")
    if isinstance(jo, JointOffsetResultRecord):
        for i, s in enumerate(arm_specs):
            joint_off[i] = jo.result_data.offsets.get(s.id, 0.0)

    link_t = link_r = None
    lo = repo.get_active(robot_id, "link_offset")
    if isinstance(lo, LinkOffsetResultRecord):
        link_t = np.zeros((n_arm, 3))
        link_r = np.zeros((n_arm, 3))
        idx_by_id = {s.id: i for i, s in enumerate(arm_specs)}
        for entry in lo.result_data.offsets:
            i = idx_by_id.get(entry.joint_id)
            if i is None:
                continue
            link_t[i] = entry.trans_m
            link_r[i] = entry.rot_rad

    sag_k_full = np.zeros(n_arm)
    sg = repo.get_active(robot_id, "sag")
    if isinstance(sg, SagOffsetResultRecord):
        idx_by_id = {s.id: i for i, s in enumerate(arm_specs)}
        for mid, k in sg.result_data.k_rad_per_m.items():
            if mid in idx_by_id:
                sag_k_full[idx_by_id[mid]] = k

    urdf = _ROBOT_DIR / robot.type / "urdf" / f"{robot.type}.urdf"
    fk_chain = FkChain(urdf, [s.name for s in arm_specs], tcp_link_name="tcp")

    applied = ["hand_eye"]
    if np.any(joint_off):
        applied.append("joint_offset")
    if link_t is not None:
        applied.append("link_offset")
    if np.any(sag_k_full):
        applied.append("sag")
    logger.info("robot=%s run=%d bundle 적용: %s", robot_id, run_id, applied)

    Ts: list[np.ndarray] = []
    for cap in repo.list_captures(run_id):
        if drop_poses and cap.pose_index in drop_poses:
            logger.info("capture #%d 명시 제외 (--drop)", cap.pose_index)
            continue
        if cap.motor_positions is None or cap.board_in_cam is None:
            logger.warning("capture #%d 결손 (PnP/joints 없음) — skip", cap.pose_index)
            continue
        q = np.array(
            [
                raw_to_rad(cap.motor_positions[s.id], spec_by_id[s.id])
                for s in arm_specs
            ],
            dtype=np.float64,
        )
        R_ee, t_ee = fk_with_sag(fk_chain, q + joint_off, link_t, link_r, sag_k_full)
        T_base_ee = np.eye(4)
        T_base_ee[:3, :3] = R_ee
        T_base_ee[:3, 3] = t_ee
        T_cam_board = np.asarray(cap.board_in_cam, dtype=np.float64)
        Ts.append(T_base_ee @ T_ee_cam @ T_cam_board)
    if len(Ts) < 3:
        raise RuntimeError(
            f"유효 캡처 {len(Ts)}장 (robot={robot_id}, 최소 3) — 더 캡처 필요"
        )
    return run_id, Ts


# ─── 리포트 ────────────────────────────────────────────────────────


def _scatter_line(label: str, stats: ObservationStats) -> str:
    r = np.array(stats.per_obs_rot_deg)
    t = np.array(stats.per_obs_t_mm)
    warn = ""
    if r.max() > SCATTER_ROT_WARN_DEG or t.max() > SCATTER_T_WARN_MM:
        warn = "  ⚠ 흩어짐 큼 — outlier 캡처 또는 보드 이동 의심"
    return (
        f"  {label}: rot mean {r.mean():.2f}° / max {r.max():.2f}°, "
        f"t mean {t.mean():.1f}mm / max {t.max():.1f}mm{warn}"
    )


def format_report(
    robot_a: str,
    robot_b: str,
    run_a: int,
    run_b: int,
    stats_a: ObservationStats,
    stats_b: ObservationStats,
    pose_b: PlanarPose,
) -> str:
    tilt = max(abs(pose_b.roll_deg), abs(pose_b.pitch_deg))
    if tilt > PLANAR_TILT_RED_DEG:
        tilt_msg = f"✗ RED — roll/pitch {tilt:.2f}° (같은 테이블이면 ≈0 이어야. 캘/보드 재점검)"
    elif tilt > PLANAR_TILT_WARN_DEG:
        tilt_msg = f"⚠ WARN — roll/pitch {tilt:.2f}° (경계 — 결과 재현 확인 권장)"
    else:
        tilt_msg = f"✓ OK — roll/pitch {tilt:.2f}° (planar 가정 정합)"
    lines = [
        "=" * 70,
        f" Cross-Calibration Report — anchor={robot_a} (원점), target={robot_b}",
        "=" * 70,
        "",
        f"[T_base←board 관측 일관성]  (run A=#{run_a}, B=#{run_b})",
        _scatter_line(f"{robot_a} ({len(stats_a.per_obs_rot_deg)}장)", stats_a),
        _scatter_line(f"{robot_b} ({len(stats_b.per_obs_rot_deg)}장)", stats_b),
        "",
        f"[{robot_b} base_pose  ({robot_a} 원점 기준)]",
        f"  x = {pose_b.x:+.4f} m",
        f"  y = {pose_b.y:+.4f} m",
        f"  z = {pose_b.z:+.4f} m",
        f"  yaw = {pose_b.yaw_deg:+.2f}°",
        "",
        f"[Planar sanity]  {tilt_msg}",
        "",
        "[robots.yaml 반영 블록 — base_pose 만 교체]",
        f"  {robot_a}:",
        "    base_pose: { x: 0.0, y: 0.0, z: 0.0, yaw_deg: 0.0 }",
        f"  {robot_b}:",
        (
            f"    base_pose: {{ x: {pose_b.x:.4f}, y: {pose_b.y:.4f}, "
            f"z: {pose_b.z:.4f}, yaw_deg: {pose_b.yaw_deg:.2f} }}"
        ),
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
        sys.stderr.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    except Exception:
        pass
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--db", required=True, help="SQLite DB 경로 (horibot.db)")
    parser.add_argument("--robot-a", default="so101_6dof_0", help="anchor (원점)")
    parser.add_argument("--robot-b", default="omx_f_0", help="배치 산출 대상")
    parser.add_argument("--run-a", type=int, default=None)
    parser.add_argument("--run-b", type=int, default=None)
    parser.add_argument(
        "--drop-a", type=int, nargs="+", default=[],
        help="robot A 에서 제외할 pose_index (예: --drop-a 7)",
    )
    parser.add_argument(
        "--drop-b", type=int, nargs="+", default=[],
        help="robot B 에서 제외할 pose_index",
    )
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname).1s] %(message)s",
    )

    robots = load_robots()
    for rid in (args.robot_a, args.robot_b):
        if rid not in robots:
            logger.error("robot %s 없음 (robot/robots.yaml)", rid)
            return 1

    _engine, session_factory = open_sqlite(args.db)
    repo = CalibrationRepository(session_factory)

    try:
        run_a, Ts_a = load_base_board_observations(
            repo, robots[args.robot_a], args.run_a, set(args.drop_a)
        )
        run_b, Ts_b = load_base_board_observations(
            repo, robots[args.robot_b], args.run_b, set(args.drop_b)
        )
    except RuntimeError as e:
        logger.error("%s", e)
        return 1

    stats_a = average_board_observations(Ts_a)
    stats_b = average_board_observations(Ts_b)
    T_a_b = compose_a_from_b(stats_a.T_base_board, stats_b.T_base_board)
    pose_b = project_planar(T_a_b)

    print(
        format_report(
            args.robot_a, args.robot_b, run_a, run_b, stats_a, stats_b, pose_b
        )
    )

    if args.output_json:
        args.output_json.write_text(
            json.dumps(
                {
                    "anchor": args.robot_a,
                    "target": args.robot_b,
                    "run_a": run_a,
                    "run_b": run_b,
                    "T_a_from_b": T_a_b.tolist(),
                    "base_pose_b": {
                        "x": pose_b.x,
                        "y": pose_b.y,
                        "z": pose_b.z,
                        "yaw_deg": pose_b.yaw_deg,
                    },
                    "planar_residual_deg": {
                        "roll": pose_b.roll_deg,
                        "pitch": pose_b.pitch_deg,
                    },
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        logger.info("결과 JSON → %s", args.output_json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
