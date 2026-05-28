"""Self-play attempt loop.

§ docs/self_play_pick.md 의 설계 구현. mode = "grasp" | "loop".

한 attempt = 한 큐브 = 한 jsonl line. 한 attempt 안에서 후보지 9방향 시도
(closed-loop self-play):

  1. detect → bin offset lookup (이전 success 데이터의 평균 보정 적용)
  2. 9방향 (중앙 + 8 neighbor) 후보지 순차 시도 — fail(SPIKE/EMPTY/DROPPED)
     이면 다음 후보지로. 첫 success 시 break.
  3. success 시 그 candidate 의 총 offset (bin lookup + candidate offset)
     을 BinOffsetCorrector 에 누적 → 같은 bin 의 다음 attempt 가 자동 보정.
  4. random drop (search pose TCP bbox 안에서 sample) 후 home 복귀.

→ "플레이 버튼 누르고 끝" 자율성 + 데이터 누적 → 자동 보정 → success rate ↑
의 닫힌 루프.

detect 실패 처리 (결정 #7): retry 3회 → search pose 순회 → 모두 fail 이면 중단.
"""

from __future__ import annotations

import json
import logging
import random
import threading
import time
from collections.abc import Sequence
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Literal

from core.common import GRIPPER_ID, GRIPPER_SETTLE
from core.robot_poses import JointAngle, list_pose_names, load_pose
from core.topic_map import Service, Topic
from core.types import TrajStatus

if TYPE_CHECKING:
    from core.base_node import BaseNode
    from core.gripper_setup import GripperSetup
    from core.joint_state_cache import JointStateCache
    from modules.calibration.loader import CalibrationData
    from modules.dynamixel.motor_config import MotorConfig


logger = logging.getLogger(__name__)


# ─── 동작 파라미터 (1차 구현; calib 후 튜닝) ─────────────────────────

HOVER_Z_OFFSET = 0.06
LIFT_Z_OFFSET = 0.08

# Grasp z 정책 (detector height 기반)
THIN_HEIGHT_THRESHOLD = 0.040
THIN_TOP_INSET = 0.005
TALL_GRASP_RATIO = 0.5

# Stage 1: descend 중 load spike 감지
LOAD_SPIKE_THRESHOLD = 300
LOAD_POLL_INTERVAL = 0.02

# Stage 2/3: gripper 잡힘/떨어짐
GRIPPER_CLOSE_CURRENT = 200
GRIPPER_HELD_THRESHOLD = 1900

# Detect 실패 처리 (결정 #7)
DETECT_RETRY_COUNT = 3
DETECT_RETRY_DELAY = 1.0
SEARCH_POSE_SETTLE = 0.5

# 후보지 (9방향) — step = detector height × ratio, 최소 5mm 보장
CANDIDATE_STEP_RATIO = 0.4
CANDIDATE_STEP_MIN = 0.005

# Workspace bin offset 학습 (closed-loop)
BIN_SIZE = 0.05  # 5cm grid
BIN_MIN_SAMPLES = 3  # bin 별 데이터 >= N 이면 보정 적용
BIN_OFFSET_FILENAME = "bin_offsets.json"

# Random drop fallback (search pose 0개 / 측정 실패 시)
DROP_X_RANGE = (0.10, 0.20)
DROP_Y_RANGE = (-0.10, 0.10)
DROP_Z = 0.05

# Workspace box (in_workspace 단순 check)
WORKSPACE_X = (-0.30, 0.30)
WORKSPACE_Y = (-0.25, 0.25)
WORKSPACE_Z = (-0.05, 0.30)

TRAJ_WAIT_TIMEOUT = 30.0


StageResult = Literal[
    "OK",        # 정상 통과
    "SPIKE",     # Stage 1: descend 중 부딪힘
    "EMPTY",     # Stage 2: close 했는데 빈손
    "DROPPED",   # Stage 3: lift 했는데 떨어짐
    "SKIPPED",   # 이전 stage 가 실패해 진행 안 함 / candidate skip
    "FAIL",      # 시스템 에러 (motion/service 호출 실패)
]


@dataclass
class CandidateResult:
    """후보지 1개 (offset_xy) 시도 결과. 한 큐브 attempt 안에 여러 개 누적."""

    offset_xy: tuple[float, float]

    s1: StageResult = "SKIPPED"
    s2: StageResult = "SKIPPED"
    s3: StageResult = "SKIPPED"

    spike_joint_id: int | None = None
    spike_load: int | None = None
    spike_baseline: int | None = None
    spike_at_z: float | None = None
    gripper_pos_after_close: int | None = None
    gripper_pos_after_lift: int | None = None

    fail_stage: int | None = None
    note: str = ""


@dataclass
class AttemptResult:
    """한 큐브 = 한 attempt = 한 jsonl line. candidates 에 후보지별 결과."""

    ts: float
    attempt_id: int
    prompt: str

    # detect
    target_xyz: list[float] | None = None
    detect_base_z: float | None = None
    detect_height: float | None = None
    grasp_z: float | None = None
    detect_retries: int = 0
    search_pose_used: str | None = None

    # closed-loop: bin 으로부터 자동 적용된 보정 (raw detected_xy 에 더해진 값)
    correction_applied: list[float] | None = None

    # 자세 snapshot (attempt 시작 시점)
    joint_raw: dict[int, int] = field(default_factory=dict)

    # 후보지별 시도 결과
    candidates: list[CandidateResult] = field(default_factory=list)

    # 요약 (success 있으면 그 candidate, 없으면 마지막 candidate)
    s1: StageResult = "SKIPPED"
    s2: StageResult = "SKIPPED"
    s3: StageResult = "SKIPPED"
    success_candidate_idx: int | None = None
    fail_stage: int | None = None
    note: str = ""


class AttemptLogger:
    """jsonl append-only. 한 attempt = 한 줄."""

    def __init__(self, log_path: Path) -> None:
        self.log_path = log_path
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def append(self, result: AttemptResult) -> None:
        with self._lock:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(asdict(result), ensure_ascii=False) + "\n")


class BinOffsetCorrector:
    """Workspace bin 별 success offset 평균. detected_xy 에 적용해서 다음 attempt
    가 점점 좋아지게 — closed-loop self-play 의 학습 component.

    저장: log_dir / bin_offsets.json (모든 세션 공유 — 누적 학습).
    bin key = (int(x/BIN_SIZE), int(y/BIN_SIZE)).
    value = list[(total_dx, total_dy)] — success 했을 때 raw detected_xy 에
            적용된 *총* 보정 (bin lookup 보정 + candidate offset 합).
    """

    def __init__(self, store_path: Path) -> None:
        self.store_path = store_path
        self._bins: dict[tuple[int, int], list[tuple[float, float]]] = {}
        self._lock = threading.Lock()
        self._load()

    def _bin_of(self, x: float, y: float) -> tuple[int, int]:
        return (int(x // BIN_SIZE), int(y // BIN_SIZE))

    def lookup(self, x: float, y: float) -> tuple[float, float] | None:
        """detected_xy bin 의 평균 offset. 데이터 부족 시 None (보정 안 함)."""
        key = self._bin_of(x, y)
        with self._lock:
            offsets = self._bins.get(key, [])
            if len(offsets) < BIN_MIN_SAMPLES:
                return None
            avg_dx = sum(o[0] for o in offsets) / len(offsets)
            avg_dy = sum(o[1] for o in offsets) / len(offsets)
        return (avg_dx, avg_dy)

    def add_success(
        self,
        detected_x: float,
        detected_y: float,
        total_offset_dx: float,
        total_offset_dy: float,
    ) -> None:
        """raw detected_xy bin 에 총 보정 offset 누적. 다음 attempt 의 lookup 평균
        이 이 누적치들의 mean 이 됨.
        """
        key = self._bin_of(detected_x, detected_y)
        with self._lock:
            self._bins.setdefault(key, []).append(
                (total_offset_dx, total_offset_dy)
            )
        self._save()

    def _load(self) -> None:
        if not self.store_path.exists():
            return
        try:
            with open(self.store_path, "r", encoding="utf-8") as f:
                raw = json.load(f)
            for k, vs in raw.items():
                ix, iy = map(int, k.split(","))
                self._bins[(ix, iy)] = [tuple(v) for v in vs]
            logger.info(
                "bin_offsets 로드: %d bins, total %d samples",
                len(self._bins),
                sum(len(vs) for vs in self._bins.values()),
            )
        except Exception as exc:
            logger.warning("bin_offsets load 실패: %s", exc)

    def _save(self) -> None:
        self.store_path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            out = {
                f"{k[0]},{k[1]}": [list(v) for v in vs]
                for k, vs in self._bins.items()
            }
        try:
            with open(self.store_path, "w", encoding="utf-8") as f:
                json.dump(out, f, indent=2)
        except Exception as exc:
            logger.warning("bin_offsets save 실패: %s", exc)


class SelfPlayRunner:
    """Self-play attempt loop. step_executor 의 _self_play 핸들러가 인스턴스화.

    step_executor 가 한 번 만든 runner 를 재사용 — _drop_bbox / _corrector
    캐시 유지.
    """

    def __init__(
        self,
        node: "BaseNode",
        joint_cache: "JointStateCache",
        arm_cfgs: list["MotorConfig"],
        calibration: "CalibrationData | None" = None,
    ) -> None:
        self._node = node
        self._joint_cache = joint_cache
        self._arm_cfgs = arm_cfgs
        self._calib = calibration

        self._traj_event = threading.Event()
        self._traj_status: str = TrajStatus.IDLE
        self._node.create_subscriber(
            Topic.MOTION_STATE_TRAJ, self._on_traj_state
        )

        self._stop_requested = threading.Event()
        self._external_stop: threading.Event | None = None

        # closed-loop state (run() 시작 시 1회 초기화, runner 재사용 시 캐시 유지)
        self._corrector: BinOffsetCorrector | None = None
        self._drop_bbox: tuple[float, float, float, float] | None = None

        # 객체별 gripper 셋업 — run() 마다 override 가능
        self._gripper_setup: "GripperSetup | None" = None

        # publish state
        self._cur_prompt: str = ""
        self._cur_attempt_id: int = 0
        self._cur_max_attempts: int = 0
        self._stats: dict[str, int] = {
            "total": 0,
            "success": 0,
            "s1_pass": 0,
            "s2_pass": 0,
            "s3_pass": 0,
        }
        self._last_result_dict: dict | None = None

    def _publish_state(self, current_stage: str) -> None:
        try:
            self._node.publish(
                Topic.SELF_PLAY_STATE,
                {
                    "prompt": self._cur_prompt,
                    "attempt_id": self._cur_attempt_id,
                    "max_attempts": self._cur_max_attempts,
                    "current_stage": current_stage,
                    "last_result": self._last_result_dict,
                    "stats": dict(self._stats),
                },
            )
        except Exception as exc:
            logger.warning("self-play state publish 실패: %s", exc)

    def request_stop(self) -> None:
        self._stop_requested.set()

    def _is_stop(self) -> bool:
        if self._stop_requested.is_set():
            return True
        if self._external_stop is not None and self._external_stop.is_set():
            return True
        return False

    def _on_traj_state(self, data: dict) -> None:
        status = data.get("status", "")
        self._traj_status = status
        if status in (TrajStatus.DONE, TrajStatus.FAILED, TrajStatus.STOPPED):
            self._traj_event.set()

    # ─── Public: 진입점 ─────────────────────────────────────────────

    def run(
        self,
        prompt: str,
        max_attempts: int,
        log_path: Path,
        stop_event: threading.Event | None = None,
        gripper_setup: "GripperSetup | None" = None,
    ) -> bool:
        logger.info(
            "self-play 시작 prompt='%s' max_attempts=%d log=%s gripper=%s",
            prompt, max_attempts, log_path, gripper_setup,
        )

        # state 초기화
        self._external_stop = stop_event
        self._gripper_setup = gripper_setup
        self._cur_prompt = prompt
        self._cur_max_attempts = max_attempts
        self._cur_attempt_id = 0
        self._stats = {
            "total": 0, "success": 0,
            "s1_pass": 0, "s2_pass": 0, "s3_pass": 0,
        }
        self._last_result_dict = None
        self._publish_state("starting")

        # closed-loop: bin corrector 초기화 (log_dir 공유 — 세션 간 학습 누적)
        if self._corrector is None:
            self._corrector = BinOffsetCorrector(
                log_path.parent / BIN_OFFSET_FILENAME
            )

        search_poses = list_pose_names("search_")
        logger.info("search poses: %s", search_poses or "(없음)")

        # random drop bbox 1회 측정 (runner 재사용 시 캐시 유지)
        if self._drop_bbox is None and search_poses:
            self._publish_state("measuring_drop_bbox")
            self._drop_bbox = self._measure_drop_bbox(search_poses)
            if self._drop_bbox:
                logger.info(
                    "drop bbox: x[%.3f, %.3f] y[%.3f, %.3f]",
                    *self._drop_bbox,
                )
            else:
                logger.warning("drop bbox 측정 실패 — fallback (하드코드 range)")

        logger_io = AttemptLogger(log_path)

        for attempt_id in range(1, max_attempts + 1):
            self._cur_attempt_id = attempt_id

            if self._is_stop():
                logger.info("self-play 중단 (attempt %d/%d)", attempt_id, max_attempts)
                self._publish_state("stopped")
                return True

            result = self._run_attempt(attempt_id, prompt, search_poses)
            logger_io.append(result)

            # 통계 갱신
            self._stats["total"] += 1
            if result.s1 == "OK":
                self._stats["s1_pass"] += 1
            if result.s2 == "OK":
                self._stats["s2_pass"] += 1
            if result.s3 == "OK":
                self._stats["s3_pass"] += 1
                self._stats["success"] += 1
            self._last_result_dict = asdict(result)
            self._publish_state("attempt_done")

            # detect 모두 실패 → task 중단
            if result.note == "detect_failed_all":
                logger.warning("detect 모두 실패 — task 중단 (attempt %d)", attempt_id)
                self._publish_state("halted")
                return False

            # 성공 → random drop (bbox 안에서 sample, 다양한 자세 데이터 누적)
            if result.s3 == "OK":
                self._publish_state("dropping")
                self._random_drop()

            # 다음 시도 전 home 복귀
            self._publish_state("returning_home")
            self._move_to_pose("home")

        logger.info(
            "self-play 종료. 시도=%d 성공=%d (%.1f%%)",
            max_attempts, self._stats["success"],
            100.0 * self._stats["success"] / max(1, max_attempts),
        )
        self._publish_state("done")
        return True

    # ─── 1 attempt = 1 큐브 (후보지 9방향 시도) ─────────────────────

    def _run_attempt(
        self,
        attempt_id: int,
        prompt: str,
        search_poses: list[str],
    ) -> AttemptResult:
        result = AttemptResult(
            ts=time.time(),
            attempt_id=attempt_id,
            prompt=prompt,
        )

        # 1) detect (+ recovery)
        self._publish_state("detecting")
        detect_data, retries, search_used = self._detect_with_recovery(
            prompt, search_poses,
        )
        result.detect_retries = retries
        result.search_pose_used = search_used

        if detect_data is None:
            result.note = "detect_failed_all"
            return result

        pos = detect_data["position"]
        target = (float(pos[0]), float(pos[1]), float(pos[2]))
        base_z = float(detect_data.get("base_z", 0.0))
        height = float(detect_data.get("height", 0.020))
        result.target_xyz = list(target)
        result.detect_base_z = base_z
        result.detect_height = height

        # Grasp z 정책
        if height < THIN_HEIGHT_THRESHOLD:
            grasp_z = target[2] - THIN_TOP_INSET
        else:
            grasp_z = base_z + height * TALL_GRASP_RATIO
        result.grasp_z = grasp_z

        logger.info(
            "[#%d] target xy=(%.3f, %.3f) base_z=%.3f h=%.3f → grasp_z=%.3f",
            attempt_id, target[0], target[1], base_z, height, grasp_z,
        )

        # 바닥 뚫는 grasp_z 면 attempt 자체 skip
        if grasp_z < base_z - 0.01:
            result.note = "grasp_below_floor"
            return result

        # 2) closed-loop: bin offset lookup → 자동 보정
        correction = (
            self._corrector.lookup(target[0], target[1])
            if self._corrector else None
        )
        if correction is not None:
            result.correction_applied = list(correction)
            logger.info(
                "[#%d] bin offset 보정 적용: (%.4f, %.4f)",
                attempt_id, correction[0], correction[1],
            )
        cor_dx, cor_dy = (correction or (0.0, 0.0))
        corrected_x = target[0] + cor_dx
        corrected_y = target[1] + cor_dy

        if not self._in_workspace((corrected_x, corrected_y, grasp_z)):
            result.note = "out_of_workspace"
            return result

        # joint snapshot
        raw = self._joint_cache.get_raw_motor_positions(self._arm_cfgs)
        if raw:
            result.joint_raw = raw

        # 3) gripper open (후보지 loop 전 한 번만)
        self._publish_state("hovering")
        if not self._gripper("open"):
            result.s1 = "FAIL"
            result.fail_stage = 1
            result.note = "gripper_open_fail"
            return result
        time.sleep(GRIPPER_SETTLE)

        # 4) 후보지 9방향 generator (step = height 의 40%, 최소 5mm)
        step = max(height * CANDIDATE_STEP_RATIO, CANDIDATE_STEP_MIN)
        candidates = self._candidate_offsets(step)

        # 5) 각 candidate 순차 시도, 첫 success 에서 break
        success_idx: int | None = None
        for cand_idx, (dx, dy) in enumerate(candidates):
            if self._is_stop():
                break

            cand_xyz = (corrected_x + dx, corrected_y + dy, grasp_z)

            if not self._in_workspace(cand_xyz):
                result.candidates.append(CandidateResult(
                    offset_xy=(dx, dy), note="out_of_workspace",
                ))
                continue

            self._publish_state(f"candidate_{cand_idx}")
            cand = self._try_grasp_at(cand_xyz, grasp_z, dx, dy)
            result.candidates.append(cand)

            if cand.s3 == "OK":
                success_idx = cand_idx
                break

        # 6) attempt 요약 (success 있으면 그거, 없으면 마지막 candidate)
        if result.candidates:
            summary = (
                result.candidates[success_idx]
                if success_idx is not None
                else result.candidates[-1]
            )
            result.s1 = summary.s1
            result.s2 = summary.s2
            result.s3 = summary.s3
            result.fail_stage = summary.fail_stage
            result.success_candidate_idx = success_idx
            if not result.note:
                result.note = summary.note

        # 7) closed-loop: success 면 bin learner 에 총 offset 누적
        if success_idx is not None and self._corrector is not None:
            succ = result.candidates[success_idx]
            total_dx = cor_dx + succ.offset_xy[0]
            total_dy = cor_dy + succ.offset_xy[1]
            self._corrector.add_success(target[0], target[1], total_dx, total_dy)
            logger.info(
                "[#%d] bin 학습 누적: detected=(%.3f, %.3f) total_offset=(%.4f, %.4f)",
                attempt_id, target[0], target[1], total_dx, total_dy,
            )

        return result

    # ─── 1 후보지 시도 ──────────────────────────────────────────────

    def _try_grasp_at(
        self,
        cand_xyz: tuple[float, float, float],
        grasp_z: float,
        offset_dx: float,
        offset_dy: float,
    ) -> CandidateResult:
        """후보지 1개에서 hover → descend → close → lift 한 번 시도.

        gripper open 은 caller (_run_attempt) 가 loop 전 한 번 처리. SPIKE/EMPTY/
        DROPPED 시 다음 후보지 위해 자체에서 hover_z 까지 retreat + 필요 시 open.
        """
        cand = CandidateResult(offset_xy=(offset_dx, offset_dy))
        hover_z = grasp_z + HOVER_Z_OFFSET
        retreat = [cand_xyz[0], cand_xyz[1], hover_z]

        # Hover
        if not self._move_l(retreat):
            cand.s1 = "FAIL"
            cand.fail_stage = 1
            cand.note = "hover_fail"
            return cand

        # Stage 1: descend + spike check
        self._publish_state("descending")
        descend = [cand_xyz[0], cand_xyz[1], grasp_z]
        spike = self._descend_with_spike_check(descend)
        if spike is not None:
            cand.s1 = "SPIKE"
            cand.spike_joint_id = spike["joint_id"]
            cand.spike_load = spike["load"]
            cand.spike_baseline = spike["baseline"]
            cand.spike_at_z = spike["z"]
            cand.fail_stage = 1
            # 다음 후보지 위해 retreat (descend stop 위치에서 위로)
            self._move_l(retreat)
            return cand
        cand.s1 = "OK"

        # Stage 2: close
        self._publish_state("closing")
        if not self._gripper("close"):
            cand.s2 = "FAIL"
            cand.fail_stage = 2
            cand.note = "gripper_close_fail"
            self._move_l(retreat)
            return cand
        time.sleep(GRIPPER_SETTLE)

        held = self._held_threshold()
        gp_close = self._get_gripper_position()
        cand.gripper_pos_after_close = gp_close
        if gp_close is None or gp_close < held:
            cand.s2 = "EMPTY"
            cand.fail_stage = 2
            # 다음 후보지 위해 open + retreat
            self._gripper("open")
            self._move_l(retreat)
            return cand
        cand.s2 = "OK"

        # Stage 3: lift
        self._publish_state("lifting")
        lift = [cand_xyz[0], cand_xyz[1], grasp_z + LIFT_Z_OFFSET]
        if not self._move_l(lift):
            cand.s3 = "FAIL"
            cand.fail_stage = 3
            cand.note = "lift_move_fail"
            return cand

        gp_lift = self._get_gripper_position()
        cand.gripper_pos_after_lift = gp_lift
        if gp_lift is None or gp_lift < held:
            cand.s3 = "DROPPED"
            cand.fail_stage = 3
            # 큐브 떨어졌으니 open (다음 candidate 위해)
            self._gripper("open")
        else:
            cand.s3 = "OK"

        return cand

    def _candidate_offsets(self, step: float) -> list[tuple[float, float]]:
        """9방향 후보 offset. (0,0) 첫 + 4 cardinal + 4 diagonal."""
        return [
            (0.0, 0.0),
            (+step, 0.0),
            (-step, 0.0),
            (0.0, +step),
            (0.0, -step),
            (+step, +step),
            (+step, -step),
            (-step, +step),
            (-step, -step),
        ]

    # ─── Detect & recovery ──────────────────────────────────────────

    def _detect_with_recovery(
        self, prompt: str, search_poses: list[str],
    ) -> tuple[dict | None, int, str | None]:
        """결정 #7 의 retry → search pose 순회 정책.

        반환: (detect data | None, 누적 retry count, 사용된 search pose 이름).
        """
        for i in range(DETECT_RETRY_COUNT):
            if self._is_stop():
                return None, i, None
            data = self._call_detect(prompt)
            if data is not None:
                return data, i, None
            if i < DETECT_RETRY_COUNT - 1:
                time.sleep(DETECT_RETRY_DELAY)

        retries = DETECT_RETRY_COUNT
        for sp_name in search_poses:
            if self._is_stop():
                return None, retries, sp_name
            self._move_to_pose(sp_name)
            time.sleep(SEARCH_POSE_SETTLE)
            data = self._call_detect(prompt)
            retries += 1
            if data is not None:
                return data, retries, sp_name

        return None, retries, None

    def _call_detect(self, prompt: str) -> dict | None:
        try:
            res = self._node.call_service(
                Service.PERCEPTION_GROUNDED_DETECT,
                {"prompt": prompt},
                timeout=60.0,
            )
        except Exception as exc:
            logger.warning("grounded_detect 예외: %s", exc)
            return None
        if not res.get("success"):
            logger.info("grounded_detect 실패: %s", res.get("message"))
            return None
        data = res.get("data") or {}
        pos = data.get("position")
        if pos is None or len(pos) != 3:
            return None
        return data

    def _in_workspace(self, pos: tuple[float, float, float]) -> bool:
        x, y, z = pos
        return (
            WORKSPACE_X[0] <= x <= WORKSPACE_X[1]
            and WORKSPACE_Y[0] <= y <= WORKSPACE_Y[1]
            and WORKSPACE_Z[0] <= z <= WORKSPACE_Z[1]
        )

    # ─── Descend + spike 감지 ──────────────────────────────────────

    def _descend_with_spike_check(
        self, target_pos: list[float],
    ) -> dict | None:
        baseline = self._joint_cache.get_present_loads(self._arm_cfgs) or {}

        self._traj_event.clear()
        res = self._node.call_service(
            Service.MOTION_MOVE_L, {"position": target_pos},
        )
        if not res.get("success"):
            logger.warning("descend MoveL 호출 실패: %s", res.get("message"))
            return {
                "joint_id": -1,
                "load": 0,
                "baseline": 0,
                "z": -1.0,
            }

        spike: dict | None = None
        deadline = time.time() + TRAJ_WAIT_TIMEOUT

        while time.time() < deadline:
            if self._traj_event.is_set():
                break
            if self._is_stop():
                try:
                    self._node.call_service(Service.MOTION_STOP, {})
                except Exception:
                    pass
                break

            loads = self._joint_cache.get_present_loads(self._arm_cfgs)
            if loads:
                for cfg in self._arm_cfgs:
                    base = baseline.get(cfg.id, 0)
                    cur = loads.get(cfg.id, 0)
                    if abs(cur - base) > LOAD_SPIKE_THRESHOLD:
                        try:
                            self._node.call_service(Service.MOTION_STOP, {})
                        except Exception:
                            pass
                        tcp = self._get_current_tcp()
                        spike = {
                            "joint_id": cfg.id,
                            "load": int(cur),
                            "baseline": int(base),
                            "z": float(tcp[2]) if tcp else -1.0,
                        }
                        logger.info(
                            "spike j%d load=%d (base=%d) z=%.3f",
                            cfg.id, cur, base, spike["z"],
                        )
                        break
                if spike:
                    break

            time.sleep(LOAD_POLL_INTERVAL)

        if not self._traj_event.is_set():
            self._traj_event.wait(timeout=3.0)

        return spike

    # ─── Drop bbox 측정 (search pose TCP 1회 캐시) ──────────────────

    def _measure_drop_bbox(
        self, search_poses: list[str],
    ) -> tuple[float, float, float, float] | None:
        """search pose 들 순회하면서 TCP xy 측정 → bbox 반환.
        run() 시작 시 1회만. runner instance 재사용 시 캐시 유지 (더 안 돌림).
        """
        xs: list[float] = []
        ys: list[float] = []
        for pose_name in search_poses:
            if self._is_stop():
                return None
            if not self._move_to_pose(pose_name):
                logger.warning("drop bbox: pose 이동 실패 '%s'", pose_name)
                continue
            time.sleep(SEARCH_POSE_SETTLE)
            tcp = self._get_current_tcp()
            if tcp is None:
                logger.warning("drop bbox: TCP 가져오기 실패 '%s'", pose_name)
                continue
            xs.append(float(tcp[0]))
            ys.append(float(tcp[1]))
            logger.info(
                "search '%s' TCP xy=(%.3f, %.3f)", pose_name, tcp[0], tcp[1]
            )

        if not xs:
            return None
        return (min(xs), max(xs), min(ys), max(ys))

    # ─── Motion / gripper helpers ──────────────────────────────────

    def _move_l(self, position: list[float]) -> bool:
        self._traj_event.clear()
        res = self._node.call_service(
            Service.MOTION_MOVE_L, {"position": position},
        )
        if not res.get("success"):
            logger.warning("MoveL 실패: %s", res.get("message"))
            return False
        return self._wait_for_traj()

    def _move_j(self, joints: Sequence[JointAngle]) -> bool:
        self._traj_event.clear()
        res = self._node.call_service(
            Service.MOTION_MOVE_J, {"joints": list(joints)},
        )
        if not res.get("success"):
            logger.warning("MoveJ 실패: %s", res.get("message"))
            return False
        return self._wait_for_traj()

    def _move_to_pose(self, name: str) -> bool:
        try:
            joints = load_pose(name)
        except KeyError as exc:
            logger.warning("자세 로드 실패: %s", exc)
            return False
        return self._move_j(joints)

    def _wait_for_traj(self, timeout: float = TRAJ_WAIT_TIMEOUT) -> bool:
        triggered = self._traj_event.wait(timeout=timeout)
        if not triggered:
            logger.warning("traj wait timeout (%.0fs)", timeout)
            return False
        return self._traj_status == TrajStatus.DONE

    def _gripper(self, action: Literal["open", "close"]) -> bool:
        """gripper open/close. self._gripper_setup 이 있으면 override 적용."""
        setup = self._gripper_setup
        payload: dict = {"action": action}
        if action == "close":
            payload["current"] = (
                setup.close_current
                if setup and setup.close_current is not None
                else GRIPPER_CLOSE_CURRENT
            )
            if setup and setup.close_position is not None:
                payload["position"] = setup.close_position
        else:  # open
            if setup and setup.open_position is not None:
                payload["position"] = setup.open_position
        res = self._node.call_service(Service.MOTOR_GRIPPER, payload)
        return bool(res.get("success"))

    def _held_threshold(self) -> int:
        """잡힘/빈손 판정 threshold (gripper Present_Position 비교용)."""
        setup = self._gripper_setup
        if setup and setup.held_threshold is not None:
            return setup.held_threshold
        return GRIPPER_HELD_THRESHOLD

    def _get_gripper_position(self) -> int | None:
        return self._joint_cache.get_raw(GRIPPER_ID)

    def _get_current_tcp(self) -> list[float] | None:
        try:
            res = self._node.call_service(Service.MOTION_GET_TCP, {})
        except Exception:
            return None
        if not res.get("success"):
            return None
        return res.get("data", {}).get("position")

    def _random_drop(self) -> None:
        """drop bbox (search pose TCP 들의 xy 영역) 안에서 random sample.
        매번 search pose 안 돌림 — _drop_bbox 는 run() 시작 시 1회만 측정.
        bbox 없으면 fallback (하드코드 range).
        """
        if self._drop_bbox is not None:
            x = random.uniform(self._drop_bbox[0], self._drop_bbox[1])
            y = random.uniform(self._drop_bbox[2], self._drop_bbox[3])
        else:
            x = random.uniform(*DROP_X_RANGE)
            y = random.uniform(*DROP_Y_RANGE)
        drop_pos = [x, y, DROP_Z]
        logger.info("random drop → (%.3f, %.3f, %.3f)", *drop_pos)
        if self._move_l(drop_pos):
            self._gripper("open")
            time.sleep(GRIPPER_SETTLE)
