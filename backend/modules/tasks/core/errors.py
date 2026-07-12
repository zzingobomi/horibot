"""Task typed 예외 — "try 를 안 쓰는 게 기본" 실패 모델의 어휘.

primitive/순수 함수는 실패 시 이 계열을 raise 하고, TaskRunner 의 exception filter 가
FAILED 전환 + 사유 조립 + (움직인 robot) Motion.STOP 을 일괄 처리한다. 시나리오가
catch 하는 경우는 도메인 복구를 원할 때뿐 (예: 파지 실패 → 놓고 재시도).

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
    """모든 접근 후보가 IK 불가 — 물체가 workspace 밖이거나 자세 전멸."""

    def __init__(self, message: str = "") -> None:
        detail = f" ({message})" if message else ""
        super().__init__(
            f"모든 접근 후보 IK 불가{detail}. 물체를 로봇 쪽으로 옮긴 후 다시 실행하세요"
        )


class MotionRejected(TaskError):
    """move 서비스가 거부 (사전 IK 판정 후 불일치 등)."""

    def __init__(self, kind: str, message: str = "") -> None:
        detail = f": {message}" if message else ""
        super().__init__(f"{kind} 거부{detail}")


class GripperFailed(TaskError):
    def __init__(self, action: str) -> None:
        super().__init__(f"gripper {action} 실패")
