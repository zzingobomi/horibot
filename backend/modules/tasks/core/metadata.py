"""TaskMetadata — GET /tasks 노출용 task 자기소개 (명시적 등록).

frontend 는 이걸로 task 페이지의 대상 robot / 실행 param 폼을 구성한다.
param 스펙은 손 목록이 아니라 **RunRequest 모델에서 자동 파생** — 실제 wire 로
받는 모양 그대로라 hand-sync 드리프트가 원천 차단된다. name/robots/description
같은 비파생 정보만 명시 필드.

등록 = task 모듈 파일 상단에서 register_task(TASK_INFO) 한 줄 (import 부수효과).
bridge 주입 (apps/resolve.py) 이 task_infos() 를 읽는다.
"""

from __future__ import annotations

from dataclasses import dataclass

from pydantic import BaseModel


@dataclass(frozen=True)
class TaskParamSpec:
    """실행 param 1개 — RunRequest 필드에서 파생 (GET /tasks 노출 모양)."""

    name: str
    type: str  # "str" | "int" | "float" | "bool"
    required: bool
    default: str = ""  # 표시용 문자열 (required=False 일 때)


@dataclass(frozen=True)
class TaskMetadata:
    name: str
    robots: list[str]  # 참여 robot — frontend 가 대상 robot 파생
    description: str
    run: str  # RUN 서비스 wire 키 (frontend 폼 → 이 서비스 호출)
    params_model: type[BaseModel]  # RunRequest — param 스펙 SSOT

    def param_specs(self) -> list[TaskParamSpec]:
        specs: list[TaskParamSpec] = []
        for name, field in self.params_model.model_fields.items():
            anno = field.annotation
            type_name = getattr(anno, "__name__", str(anno))
            if type_name not in ("str", "int", "float", "bool"):
                raise TypeError(
                    f"task '{self.name}' param '{name}': GET /tasks 노출 가능 타입은 "
                    f"str/int/float/bool (got {type_name}) — 복합 타입은 계약 재고"
                )
            required = field.is_required()
            default = "" if required else str(field.get_default())
            specs.append(
                TaskParamSpec(
                    name=name, type=type_name, required=required, default=default
                )
            )
        return specs


_REGISTRY: dict[str, TaskMetadata] = {}


def register_task(meta: TaskMetadata) -> TaskMetadata:
    """task 모듈 파일 상단에서 호출 — 같은 이름 재등록은 프로그래밍 오류 (fail-fast)."""
    if meta.name in _REGISTRY:
        raise ValueError(f"task '{meta.name}' 중복 등록")
    meta.param_specs()  # 등록 시점 검증 — 미지원 param 타입 fail-fast
    _REGISTRY[meta.name] = meta
    return meta


def task_infos() -> list[TaskMetadata]:
    """등록된 task 목록 (name 정렬) — bridge GET /tasks 주입용."""
    return [_REGISTRY[name] for name in sorted(_REGISTRY)]
