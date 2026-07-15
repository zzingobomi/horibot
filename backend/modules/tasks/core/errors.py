"""Task typed 예외 — "try 를 안 쓰는 게 기본" 실패 모델의 어휘.

@step 함수/순수 함수는 실패 시 이 계열을 raise 하고, TaskRunner 의 exception filter
가 FAILED 전환 + 사유 조립 + 참여 robot Motion.STOP 을 일괄 처리한다. 시나리오가
catch 하는 경우는 도메인 복구를 원할 때뿐 (예: 파지 실패 → 놓고 재시도).

여기 두는 것은 **도메인 판정 실패** (부정적 데이터에 task 가 내리는 판정 —
후보 0개, 도달 전멸). 서비스의 기술적 실패 (IK 불능, motor state 미도달 등)는
서비스 자신이 raise → RemoteError(type, message) 로 도달하므로 여기 어휘 불필요
(옛 MotionRejected/GripperFailed 재정의 폐기 — 2026-07-13).

각 예외는 사람이 읽을 메시지를 스스로 조립한다 — UI 의 "사유 + 다음 행동" 표시가
이 문자열 하나로 성립해야 함 (침묵 금지 원칙).
"""

from __future__ import annotations


class TaskError(Exception):
    """task 도메인 실패 공통 베이스 — runner exception filter 대상."""


class DetectionNotFound(TaskError):
    """검출 후보가 없거나 prior 를 전부 탈락 — 재배치/prompt 수정 후 재시도 대상."""

    def __init__(self, prompt: str, *, candidates: int = 0, reason: str = "") -> None:
        self.prompt = prompt
        self.candidates = candidates
        detail = f" ({reason})" if reason else ""
        super().__init__(
            f"'{prompt}' 검출 실패 — 후보 {candidates}개{detail}. "
            f"물체 배치/조명 확인 후 다시 실행하세요"
        )


class NoReachableGrasp(TaskError):
    """실행 가능한 접근 후보 없음 — 도달(IK) 전멸이거나 안전(바닥/그리퍼↔물체
    충돌) 전멸 (grasping.md §1 "안전 파지 불가" 명시 실패).

    RESOLVE_REACHABLE 의 index=-1 은 **데이터** (부정적이지만 유효한 결과) —
    그걸 치명으로 판정해 raise 하는 것은 시나리오/step 의 몫.
    """

    def __init__(self, message: str = "") -> None:
        detail = f" ({message})" if message else ""
        super().__init__(
            f"실행 가능한 접근 후보 없음{detail}. 물체를 로봇 쪽으로 옮기거나 "
            "주변 장애물을 치운 후 다시 실행하세요"
        )


class GraspFailed(TaskError):
    """그리퍼가 물체를 물지 못함 (또는 물었다 이송 중 놓침) — 실제 도달 그리퍼
    위치가 fully-closed 근처(= 사이에 아무것도 없음)로 판정된 실패.

    도메인 판정 실패다: motor READ_STATE 의 raw 위치라는 **데이터**에 task 가
    "물지 못함" 판정을 내려 raise. 침묵 성공(빈 손으로 place 진행) 방지 —
    어제까지 이 게이트가 없어 못 집어도 성공처럼 진행됐다 (2026-07-14 사고).
    """

    def __init__(
        self, *, phase: str, achieved_raw: int, close_raw: int, load_raw: int | None
    ) -> None:
        self.phase = phase
        self.achieved_raw = achieved_raw
        load = f", 부하 {load_raw}" if load_raw is not None else ""
        super().__init__(
            f"파지 실패 ({phase}) — 그리퍼가 거의 완전히 닫힘 "
            f"(도달 raw {achieved_raw} ≈ close {close_raw}{load}) = 물체 없음/놓침. "
            "물체 위치·파지 폭 확인 후 다시 실행하세요"
        )
