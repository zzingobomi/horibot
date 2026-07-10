"""Motion D4 (calibration consumer) 검증.

핵심 invariant:
1. urdf_patch 의미 = FkChain link_offset 의미 (offline BA 가 추정한 그 의미).
   FkChain(patched).fk(θ) ≈ FkChain(orig).fk(θ, link_trans, link_rot) — 어긋나면
   "BA 가 추정한 보정"과 "런타임이 적용한 보정"이 달라져 캘이 통째로 무효.
2. SagCorrectedKinematics — fk 는 θ+sagΔ 로 inner 호출, ik 는 inner 결과를
   commanded 로 역변환 (FkChain 수식 SSOT).
3. MotionModule._build_kinematics — bundle → patched URDF 경로 / sag decorator /
   joint_off 파생. raw↔rad 변환에 joint_offset 반영 (양방향).
"""

from __future__ import annotations

import shutil
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import pytest

from apps.config import load_robots
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.calibration.contract import (
    Calibration,
    CalibrationActivated,
    CalibrationBundle,
    JointOffsetResultData,
    JointOffsetResultRecord,
    LinkOffsetEntry,
    LinkOffsetResultData,
    LinkOffsetResultRecord,
    SagOffsetResultData,
    SagOffsetResultRecord,
    SnapshotBundleRequest,
)
from modules.motion import units
from modules.motion.fk_chain import FkChain
from modules.motion.module import MotionModule
from modules.motion.sag_kinematics import SagCorrectedKinematics
from modules.motion.urdf_patch import patch_urdf_link_offsets
from modules.motor.contract import JointState, MotorKind

_SO101 = "so101_6dof_0"
_SRC_URDF = (
    Path(__file__).resolve().parents[2].parent
    / "robot" / "so101_6dof" / "urdf" / "so101_6dof.urdf"
)


@pytest.fixture
def robot():
    return load_robots()[_SO101]


@pytest.fixture
def arm(robot):
    return [s for s in robot.motors if s.kind != MotorKind.GRIPPER]


@pytest.fixture
def urdf_copy(tmp_path: Path) -> Path:
    """원본 URDF 를 tmp 로 — patch 산출물이 repo 트리를 안 건드리게."""
    dst = tmp_path / _SRC_URDF.name
    shutil.copy(_SRC_URDF, dst)
    return dst


# ─── 1. urdf_patch ↔ FkChain 의미 등가 (골드) ────────────────────


def test_urdf_patch_matches_fk_chain_semantics(urdf_copy: Path, arm):
    names = [s.name for s in arm]
    n = len(names)
    rng = np.random.default_rng(7)
    link_t = rng.uniform(-0.004, 0.004, size=(n, 3))  # 실 BA 산출 크기(≈mm) 대역
    link_r = rng.uniform(-0.02, 0.02, size=(n, 3))  # rad rotvec

    patched = patch_urdf_link_offsets(
        urdf_copy,
        _SO101,
        {names[i]: (list(link_t[i]), list(link_r[i])) for i in range(n)},
    )

    chain_orig = FkChain(urdf_copy, names)
    chain_patched = FkChain(patched, names)

    for _ in range(5):
        theta = rng.uniform(-1.0, 1.0, size=n)
        R_ref, t_ref = chain_orig.fk(theta, link_t, link_r)
        R_p, t_p = chain_patched.fk(theta)
        # 뒤집으면 잡힘: 회전 합성 순서/프레임을 바꾸면 mm~cm 급 차이
        assert np.allclose(t_p, t_ref, atol=1e-9), f"translation 불일치: {t_p - t_ref}"
        assert np.allclose(R_p, R_ref, atol=1e-9)


def test_urdf_patch_rejects_unknown_joint(urdf_copy: Path):
    with pytest.raises(ValueError, match="URDF 에 없는 joint"):
        patch_urdf_link_offsets(urdf_copy, _SO101, {"nope": ([0, 0, 0], [0, 0, 0])})


# ─── 2. SagCorrectedKinematics ───────────────────────────────────


class _RecordingKin:
    """inner Kinematics 스텁 — fk/ik 입력 기록."""

    def __init__(self) -> None:
        self.last_fk_input: list[float] | None = None
        self.ik_result: list[float] = [0.1, 0.5, -0.4, 0.2, 0.1, 0.0]

    def initialize(self) -> None: ...
    def close(self) -> None: ...

    @property
    def dof(self) -> int:
        return 6

    @property
    def tcp_link_name(self) -> str:
        return "tcp"

    def fk(self, joint_angles):
        self.last_fk_input = list(joint_angles)
        return (0.1, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0)

    def ik(self, target_position, target_quaternion, current_joint_angles=None, restarts=None):
        return list(self.ik_result)

    def fk_to_matrix(self, joint_angles):
        self.last_fk_input = list(joint_angles)
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], (0.0, 0.0, 0.0)

    def joint_limits(self, n=None):
        return [(-3.14, 3.14)] * 6

    def self_collision(self, joint_angles) -> bool:
        return False


def test_sag_fk_calls_inner_with_actual_angles(urdf_copy: Path, arm):
    names = [s.name for s in arm]
    chain = FkChain(urdf_copy, names)
    k, idx = [0.12], [1]  # shoulder
    inner = _RecordingKin()
    kin = SagCorrectedKinematics(inner, chain, k, idx)

    theta = [0.0, 0.5, -0.5, 0.0, 0.3, 0.0]
    expected = chain.apply_gravity_sag(np.asarray(theta), np.asarray(k), idx)
    delta = float(expected[1] - theta[1])
    assert abs(delta) > 1e-5, "이 자세에서 sag torque 가 0 이면 test 무의미"

    kin.fk(theta)
    assert inner.last_fk_input is not None
    assert np.allclose(inner.last_fk_input, expected)


def test_sag_ik_returns_commanded_angles(urdf_copy: Path, arm):
    names = [s.name for s in arm]
    chain = FkChain(urdf_copy, names)
    k, idx = [0.12], [1]
    inner = _RecordingKin()
    kin = SagCorrectedKinematics(inner, chain, k, idx)

    sol = kin.ik((0.1, 0.0, 0.2), None, None)
    assert sol is not None
    expected = chain.actual_to_commanded(np.asarray(inner.ik_result), np.asarray(k), idx)
    assert np.allclose(sol, expected)
    # round-trip 일관성: 명령 sol 을 다시 fk 하면 inner 는 actual(≈ik_result) 을 받음
    kin.fk(sol)
    assert inner.last_fk_input is not None
    assert np.allclose(inner.last_fk_input, inner.ik_result, atol=1e-4)  # 1차 근사


# ─── 3. MotionModule bundle 소비 ─────────────────────────────────


class _DummyRuntime:
    def __init__(self) -> None:
        self.published: list[tuple[str, object]] = []

    def publish(self, key, event) -> None:
        self.published.append((str(key), event))

    async def call(self, *a, **k):  # _build_kinematics 직접 호출 경로에선 미사용
        raise AssertionError("call 불필요")


def _record_kw(robot_id: str) -> dict:
    return {"run_id": 1, "robot_id": robot_id, "created_at": datetime.now(UTC)}


def _make_bundle(robot_id: str, arm) -> CalibrationBundle:
    return CalibrationBundle(
        robot_id=robot_id,
        joint_offset=JointOffsetResultRecord(
            **_record_kw(robot_id),
            result_data=JointOffsetResultData(
                offsets={arm[2].id: 0.113, arm[4].id: -0.09}, method="test"
            ),
        ),
        link_offset=LinkOffsetResultRecord(
            **_record_kw(robot_id),
            result_data=LinkOffsetResultData(
                offsets=[
                    LinkOffsetEntry(
                        joint_id=arm[0].id, trans_m=[0.001, 0, 0], rot_rad=[0, 0.002, 0]
                    )
                ],
                method="test",
            ),
        ),
        sag=SagOffsetResultRecord(
            **_record_kw(robot_id),
            result_data=SagOffsetResultData(
                k_rad_per_m={arm[1].id: 0.116, arm[2].id: -0.013}, method="test"
            ),
        ),
    )


def _make_module(robot, arm, urdf: Path, factory) -> MotionModule:
    return MotionModule(
        runtime=_DummyRuntime(),  # type: ignore[arg-type]
        robot_id=_SO101,
        kinematics_factory=factory,
        urdf_path=urdf,
        arm_specs=arm,
        joint_max_velocity=[1.0] * 6,
        joint_max_acceleration=[1.0] * 6,
        joint_max_jerk=[1.0] * 6,
        cartesian_max_velocity=0.1,
        cartesian_max_acceleration=0.5,
        cartesian_max_jerk=1.0,
    )


def test_build_kinematics_applies_all_three(robot, arm, urdf_copy: Path):
    factory_paths: list[Path] = []

    def factory(p: Path):
        factory_paths.append(Path(p))
        return _RecordingKin()

    mod = _make_module(robot, arm, urdf_copy, factory)
    mod._build_kinematics(_make_bundle(_SO101, arm))

    # link_offset → factory 가 patched URDF 를 받음 (원본 아님)
    assert factory_paths and factory_paths[0] != urdf_copy
    assert factory_paths[0].name.endswith(".calibrated.urdf")
    # sag → decorator 로 감쌈
    assert isinstance(mod._kin, SagCorrectedKinematics)
    # joint_offset → arm 순서 배열 (없는 joint 는 0)
    assert mod._joint_off is not None
    assert mod._joint_off[2] == pytest.approx(0.113)
    assert mod._joint_off[4] == pytest.approx(-0.09)
    assert mod._joint_off[0] == 0.0


def test_build_kinematics_without_bundle_is_uncorrected(robot, arm, urdf_copy: Path):
    factory_paths: list[Path] = []

    def factory(p: Path):
        factory_paths.append(Path(p))
        return _RecordingKin()

    mod = _make_module(robot, arm, urdf_copy, factory)
    mod._build_kinematics(None)
    assert factory_paths == [urdf_copy]  # 원본 그대로
    assert isinstance(mod._kin, _RecordingKin)  # decorator 없음
    assert mod._joint_off is None


def test_joint_offset_round_trip_through_module(robot, arm, urdf_copy: Path):
    """raw→rad(+off) 와 rad→raw(−off) 가 대칭 — TCP_STATE.joints 를 그대로 MoveJ
    target 으로 되돌리면(waypoint replay) 같은 raw 로 감."""
    mod = _make_module(robot, arm, urdf_copy, lambda p: _RecordingKin())
    mod._build_kinematics(_make_bundle(_SO101, arm))

    raw_in = [2041, 2342, 903, 2846, 2120, 3122, 2048]  # 실 캡처 자세 + gripper
    mod.on_motor_state(
        JointState(
            robot_id=_SO101, seq=1, timestamp_unix=0.0, positions_raw=raw_in
        )
    )
    assert mod._latest_arm_rad is not None
    # 보정 반영 확인 — joint3(index 2) 에 +0.113
    plain = units.joints_raw_to_rad(raw_in[:6], arm)
    assert mod._latest_arm_rad[2] == pytest.approx(plain[2] + 0.113)

    # 그 각을 그대로 명령 → 같은 raw 로 복귀 (offset 이중적용/미적용이면 어긋남)
    mod._publish_cmd(list(mod._latest_arm_rad))
    runtime = mod.runtime
    assert runtime.published  # type: ignore[union-attr]
    _, cmd = runtime.published[-1]  # type: ignore[union-attr]
    assert list(cmd.positions_raw) == raw_in[:6]  # type: ignore[attr-defined]


# ─── 4. 부팅 순서 수렴 e2e — owner 늦은 부팅 (2026-07-07 근본 수정) ──


def _make_bundle_with_ids(robot_id: str, arm, base_id: int) -> CalibrationBundle:
    """signature 비교 가능한 bundle (result id 부여) — stale 감지 테스트용."""
    def kw(offset: int) -> dict:
        return {**_record_kw(robot_id), "id": base_id + offset}

    return CalibrationBundle(
        robot_id=robot_id,
        joint_offset=JointOffsetResultRecord(
            **kw(0),
            result_data=JointOffsetResultData(
                offsets={arm[2].id: 0.113, arm[4].id: -0.09}, method="test"
            ),
        ),
        sag=SagOffsetResultRecord(
            **kw(1),
            result_data=SagOffsetResultData(
                k_rad_per_m={arm[1].id: 0.116}, method="test"
            ),
        ),
    )


async def test_motion_converges_when_calibration_owner_boots_later(
    robot, arm, urdf_copy: Path
):
    """THE 시나리오 — 분산에서 PC(calibration) 없이 motion 부팅:
    ① 무보정으로 뜸 (calibration_applied=False, 조용히 X — 상태 표면화)
    ② calibration owner 늦은 부팅 → liveliness → mirror → live 적용 (재시작 없이 수렴)
    ③ 적용 후 캘 변경 (ACTIVATED) → 재빌드 X + calibration_stale=True (변경은 재부팅)
    """
    import asyncio
    import time as _time

    _cfg = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}
    ep = "tcp/127.0.0.1:17564"

    class _CalOwner:
        def __init__(self, runtime: ModuleRuntime, bundle: CalibrationBundle):
            self.runtime = runtime
            self._bundle = bundle

        @service(Calibration.Service.SNAPSHOT_BUNDLE)
        def snapshot(self, req: SnapshotBundleRequest) -> CalibrationBundle:
            return self._bundle

        def change(self, bundle: CalibrationBundle) -> None:
            self._bundle = bundle
            self.runtime.publish(
                Calibration.Event.ACTIVATED,
                CalibrationActivated(
                    robot_id=bundle.robot_id,
                    result_id=999,
                    kind="joint_offset",
                ),
            )

    async def _wait(pred, timeout: float = 6.0) -> bool:
        deadline = _time.time() + timeout
        while _time.time() < deadline:
            if pred():
                return True
            await asyncio.sleep(0.05)
        return False

    t_motion = ZenohTransport({**_cfg, "listen": [ep]})
    rt_motion = Runtime(t_motion)
    rt_motion.mirror_snapshot_timeout = 0.3
    t_cal: ZenohTransport | None = None
    rt_cal: Runtime | None = None

    mod = rt_motion.add_module(
        MotionModule,
        robot_id=_SO101,
        kinematics_factory=lambda p: _RecordingKin(),
        urdf_path=urdf_copy,
        arm_specs=arm,
        joint_max_velocity=[1.0] * 6,
        joint_max_acceleration=[1.0] * 6,
        joint_max_jerk=[1.0] * 6,
        cartesian_max_velocity=0.1,
        cartesian_max_acceleration=0.5,
        cartesian_max_jerk=1.0,
    )
    try:
        # ① calibration owner 없이 부팅 — 무보정 + 상태 표면화
        await rt_motion.start()
        assert mod._calibration_applied is False
        assert mod._joint_off is None
        assert isinstance(mod._kin, _RecordingKin)  # decorator 없음

        # ② owner 늦은 부팅 → event publish 없이 liveliness 만으로 수렴
        bundle_v1 = _make_bundle_with_ids(_SO101, arm, base_id=1)
        t_cal = ZenohTransport({**_cfg, "connect": [ep]})
        rt_cal = Runtime(t_cal)
        owner = rt_cal.add_module(_CalOwner, bundle=bundle_v1)
        await rt_cal.start()

        assert await _wait(lambda: mod._calibration_applied), (
            "owner 늦은 부팅 후 calibration 미적용 — liveliness 수렴 경로 죽음"
        )
        assert mod._joint_off is not None
        assert mod._joint_off[2] == pytest.approx(0.113)
        assert isinstance(mod._kin, SagCorrectedKinematics)  # sag live 적용
        assert mod._calibration_stale is False

        # ③ 적용 후 캘 변경 → 재빌드 X + stale 표시 (변경은 재부팅 결정 유지)
        kin_before = mod._kin
        owner.change(_make_bundle_with_ids(_SO101, arm, base_id=100))
        assert await _wait(lambda: mod._calibration_stale), (
            "캘 변경 후 stale 미표시"
        )
        assert mod._kin is kin_before, "변경인데 live 재빌드됨 — 정책 위반"
    finally:
        await rt_motion.stop()
        if rt_cal is not None:
            await rt_cal.stop()
        t_motion.close()
        if t_cal is not None:
            t_cal.close()
