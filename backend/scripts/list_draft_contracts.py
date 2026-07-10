"""아직 안 굳은 계약(DraftModel payload)을 나열한다.

DraftModel 은 "탐색 단계라 타입 미확정" 을 나타내는 명시적 마커(framework/contract/
model.py). 이 스크립트는 MODULE_REGISTRY 전 module 을 introspect 해서 payload 가
DraftModel 인 wire(service/stream/event)를 모아 보여준다 — 손 grep 대신 기계적으로
"정리해야 할 임시 계약" 을 가시화하는 audit 도구.

기본은 리포트(항상 exit 0). `--strict` 는 draft 가 하나라도 남아있으면 exit 1 —
릴리스 게이트/CI 에서 "draft 0" 를 강제하고 싶을 때만. 평소 dev 브랜치 CI 는
리포트(비차단)로 두어 정상 탐색을 막지 않는다.

CLI:
  uv run python scripts/list_draft_contracts.py
  uv run python scripts/list_draft_contracts.py --strict   # draft 있으면 exit 1
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from framework.contract.model import DraftModel  # noqa: E402


def _is_draft(cls: type | None) -> bool:
    return cls is not None and isinstance(cls, type) and issubclass(cls, DraftModel)


def collect_draft_wires() -> list[tuple[str, str, str]]:
    """(category, wire_key, draft_payload_names) 목록. category = service/stream/event."""
    from apps.registry import MODULE_REGISTRY, load_module_class
    from framework.runtime.snapshot import build_snapshot_from_classes

    classes = [load_module_class(name) for name in MODULE_REGISTRY]
    snapshot = build_snapshot_from_classes(classes)

    out: list[tuple[str, str, str]] = []
    for wire_key, (req_cls, res_cls) in sorted(snapshot.services.items()):
        drafts = [c.__name__ for c in (req_cls, res_cls) if _is_draft(c)]
        if drafts:
            out.append(("service", wire_key, ", ".join(drafts)))
    for wire_key, payload_cls in sorted(snapshot.topics.items()):
        if _is_draft(payload_cls):
            # wire_key prefix (stream/event) 로 category 표기
            category = wire_key.split("/", 1)[0]
            out.append((category, wire_key, payload_cls.__name__))
    return out


def main() -> None:
    # Windows 콘솔(cp949)이 em-dash/화살표 등 유니코드를 못 찍는 함정 회피.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="draft 계약이 하나라도 있으면 exit 1 (릴리스 게이트)",
    )
    args = parser.parse_args()

    drafts = collect_draft_wires()
    if not drafts:
        print("draft 계약 없음 — 모든 wire payload 가 확정 타입입니다.")
        sys.exit(0)

    print(f"draft 계약 {len(drafts)}건 (아직 타입 미확정 — 정리 대상):\n")
    width = max(len(cat) for cat, _, _ in drafts)
    for category, wire_key, payloads in drafts:
        print(f"  [{category:<{width}}] {wire_key}  →  {payloads}")

    if args.strict:
        print("\n--strict: draft 가 남아있어 실패 처리합니다.")
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
