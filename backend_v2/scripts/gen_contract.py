"""TS contract generator — backend_v2 modules/*/{contract,module}.py introspect.

backend_v2_modules.md §8 — contract.py 가 두 generator 의 SSOT. 본 script =
frontend TS 결합 generator (옛 frontend/scripts/gen-contract.mjs 대체).

추출 source:
  - contract.py — outer Module class (Motor/Camera/Motion/Bridge) + nested
    Service/Stream/Event StrEnum + BaseModel + StrEnum (nested 제외)
  - module.py — @publishes / @service / @subscriber spec 의 class attribute
    (_publishes_spec / _service_spec / _subscriber_spec) → key↔payload 매핑

emit:
  - TS interface (BaseModel)
  - TS enum-style const (StrEnum) + value union type
  - Topic + TopicPayloadMap (Stream + Event 합침, 옛 frontend 호환)
  - ServiceKey + ServiceMap

usage:
    cd backend_v2
    uv run --no-sync python scripts/gen_contract.py \
        --out ../frontend/src/api/generated/contract.ts
"""

from __future__ import annotations

import argparse
import importlib
import inspect
import sys
import types as _types
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from types import ModuleType
from typing import Any, Literal, Union, get_args, get_origin

# backend_v2 root 를 path 박은 후 framework / modules import
_BACKEND_V2_ROOT = Path(__file__).resolve().parent.parent
if str(_BACKEND_V2_ROOT) not in sys.path:
    sys.path.insert(0, str(_BACKEND_V2_ROOT))

from pydantic import BaseModel
from pydantic_core import PydanticUndefined

from framework.contract.publisher import get_publishes_spec
from framework.contract.service import get_service_spec
from framework.contract.subscriber import get_subscriber_spec

# ─── module discovery ──────────────────────────────────────────────


# nested class 이름 (outer class 안 박힌 StrEnum) — TS top-level emit 제외.
NESTED_CATEGORIES = ("Service", "Stream", "Event")
# outer Module class 이름 — module name PascalCase. discovery 시 추론.
# 명시 mapping 박지 X (관습: `Motor` / `Camera` / `Motion` / `Bridge`).


def pascal(name: str) -> str:
    return "".join(p.capitalize() for p in name.split("_"))


def discover_module_dirs(modules_root: Path) -> list[str]:
    out: list[str] = []
    for child in sorted(modules_root.iterdir()):
        if not child.is_dir():
            continue
        if (child / "contract.py").is_file():
            out.append(child.name)
    return out


# ─── catalog dataclass ─────────────────────────────────────────────


@dataclass
class StrEnumEntry:
    name: str  # "MotorCapability"
    module: str  # "motor" — debug
    members: list[tuple[str, str]]  # [(NAME, value)]


@dataclass
class BaseModelEntry:
    name: str  # 원본 (충돌 자리만 prefix)
    module: str
    cls: type[BaseModel]


@dataclass
class KeyEntry:
    """Service / Stream / Event key 1 개."""

    const_name: str  # "MOTION_JOG_J" (SCREAMING_SNAKE, module prefix)
    key: str  # "stream/motion/{robot_id}/jog_j"
    category: Literal["service", "stream", "event"]
    req_cls: type[BaseModel] | None
    res_cls: type[BaseModel] | None  # service 만


@dataclass
class Catalog:
    enums: dict[str, StrEnumEntry] = field(default_factory=dict)
    models: dict[str, BaseModelEntry] = field(default_factory=dict)
    keys: list[KeyEntry] = field(default_factory=list)
    # type cls → emit name (충돌 prefix 후) — 모든 reference resolution 통과
    type_name: dict[type, str] = field(default_factory=dict)


# ─── load + extract ────────────────────────────────────────────────


def load_contract(mod_name: str) -> ModuleType:
    return importlib.import_module(f"modules.{mod_name}.contract")


def load_module_py_files(mod_dir: Path, mod_name: str) -> list[ModuleType]:
    """module dir root 의 *.py 다 import (contract.py / __init__.py 제외).
    framework spec (@publishes / @service / @subscriber) 박힌 class 의 source.

    한 module 자리 단일 file (module.py) 가정 안 함 — CameraDecoded 같은 자리
    decoded.py 별 file. drivers/ subdir 자리는 scan 안 함 (helper, spec 없음)."""
    out: list[ModuleType] = []
    for py in sorted(mod_dir.glob("*.py")):
        if py.name in ("__init__.py", "contract.py"):
            continue
        full = f"modules.{mod_name}.{py.stem}"
        try:
            out.append(importlib.import_module(full))
        except ModuleNotFoundError as e:
            # 진짜 file 자리 못 찾는 경우만 silent skip — 그 외 dep 누락 등은 fail-fast
            if getattr(e, "name", None) == full:
                continue
            raise
    return out


def find_outer_class(contract_mod: ModuleType, prefix: str) -> type | None:
    obj = getattr(contract_mod, prefix, None)
    if isinstance(obj, type) and not issubclass(obj, BaseModel):
        return obj
    return None


def collect_enums(
    contract_mod: ModuleType, exclude: set[str]
) -> dict[str, StrEnumEntry]:
    """contract.py 안 top-level StrEnum (nested 제외)."""
    out: dict[str, StrEnumEntry] = {}
    for name, obj in inspect.getmembers(contract_mod, inspect.isclass):
        if obj.__module__ != contract_mod.__name__:
            continue
        if name in exclude:
            continue
        if not issubclass(obj, Enum) or not issubclass(obj, str):
            continue
        if obj is str:
            continue
        members = [(m.name, str(m.value)) for m in obj]
        out[name] = StrEnumEntry(name=name, module=contract_mod.__name__, members=members)
    return out


def collect_models(contract_mod: ModuleType) -> dict[str, BaseModelEntry]:
    out: dict[str, BaseModelEntry] = {}
    for name, obj in inspect.getmembers(contract_mod, inspect.isclass):
        if obj.__module__ != contract_mod.__name__:
            continue
        if not issubclass(obj, BaseModel) or obj is BaseModel:
            continue
        out[name] = BaseModelEntry(name=name, module=contract_mod.__name__, cls=obj)
    return out


def collect_keys_from_outer(outer: type | None, mod_prefix: str) -> list[KeyEntry]:
    """Service/Stream/Event nested StrEnum → KeyEntry (payload mapping 은
    module.py introspect 자리에서 채움 — 여기는 key + category 뼈대만)."""
    if outer is None:
        return []
    out: list[KeyEntry] = []
    for category in NESTED_CATEGORIES:
        inner = getattr(outer, category, None)
        if not isinstance(inner, type) or not issubclass(inner, Enum):
            continue
        cat: Literal["service", "stream", "event"] = category.lower()  # type: ignore[assignment]
        for member in inner:
            out.append(
                KeyEntry(
                    const_name=f"{mod_prefix.upper()}_{member.name}",
                    key=str(member.value),
                    category=cat,
                    req_cls=None,
                    res_cls=None,
                )
            )
    return out


def fill_payload_from_module_py(
    keys_by_value: dict[str, KeyEntry], module_py: ModuleType
) -> None:
    """module.py 안 @publishes / @service / @subscriber spec 의 payload 매핑.
    한 key 가 여러 source 에서 발견되어도 동일 cls 여야 — 다르면 fail-fast."""
    for cls_name, cls in inspect.getmembers(module_py, inspect.isclass):
        if cls.__module__ != module_py.__name__:
            continue

        # @publishes — class-level spec
        pub_spec = get_publishes_spec(cls)
        if pub_spec is not None:
            for wire_key, event_cls in pub_spec.pairs:
                _assign_payload(keys_by_value, wire_key, event_cls, kind="publish")

        # @service / @subscriber — method-level spec
        for _, method in inspect.getmembers(cls, inspect.isfunction):
            srv_spec = get_service_spec(method)
            if srv_spec is not None:
                _assign_service_payload(
                    keys_by_value, srv_spec.wire_key, srv_spec.req_cls, srv_spec.res_cls
                )
            sub_spec = get_subscriber_spec(method)
            if sub_spec is not None:
                _assign_payload(
                    keys_by_value, sub_spec.wire_key, sub_spec.event_cls, kind="subscribe"
                )


def _assign_payload(
    keys_by_value: dict[str, KeyEntry], key: str, cls: type[BaseModel], *, kind: str
) -> None:
    entry = keys_by_value.get(key)
    if entry is None:
        # key 가 어디서도 catalog 박혀있지 않음 — Stream 의 외부 owner 자리 (예:
        # Motor.Stream.COMMAND 를 Motion 이 publish). 단 contract.py 에 enum 박혀있어야
        # generator 가 알 수 있음. 박혀있는데 mismatch 면 logic 문제.
        raise RuntimeError(
            f"{kind} spec key {key!r} 가 contract.py 어느 module 의 "
            "Service/Stream/Event nested StrEnum 에도 박혀있지 않음 "
            "(generator 가 키 정의 못 찾음)."
        )
    if entry.req_cls is None:
        entry.req_cls = cls
    elif entry.req_cls is not cls:
        # 다른 module 의 같은 key 에 다른 payload 매핑 — backend_v2 spec invariant 깨짐.
        raise RuntimeError(
            f"key {key!r} 의 payload mismatch: "
            f"{entry.req_cls.__name__} vs {cls.__name__}"
        )


def _assign_service_payload(
    keys_by_value: dict[str, KeyEntry],
    key: str,
    req_cls: type[BaseModel],
    res_cls: type[BaseModel],
) -> None:
    entry = keys_by_value.get(key)
    if entry is None:
        raise RuntimeError(
            f"service spec key {key!r} 가 contract.py 의 nested "
            "Service StrEnum 에 박혀있지 않음."
        )
    if entry.category != "service":
        raise RuntimeError(
            f"key {key!r} 가 contract.py 에선 {entry.category} 인데 "
            "@service 가 박힘 — 분류 mismatch."
        )
    if entry.req_cls is None:
        entry.req_cls = req_cls
        entry.res_cls = res_cls
    elif entry.req_cls is not req_cls or entry.res_cls is not res_cls:
        raise RuntimeError(
            f"service {key!r} payload mismatch: "
            f"req {entry.req_cls.__name__} vs {req_cls.__name__}, "
            f"res {entry.res_cls.__name__ if entry.res_cls else 'None'} "
            f"vs {res_cls.__name__}"
        )


# ─── name conflict resolution ─────────────────────────────────────


def resolve_names(catalog: Catalog) -> None:
    """충돌 자리만 module prefix. 중복 없는 자리는 원본 이름."""
    seen: dict[str, list[BaseModelEntry | StrEnumEntry]] = {}
    for e in catalog.models.values():
        seen.setdefault(e.name, []).append(e)
    for e in catalog.enums.values():
        seen.setdefault(e.name, []).append(e)

    for orig_name, entries in seen.items():
        if len(entries) == 1:
            e = entries[0]
            if isinstance(e, BaseModelEntry):
                catalog.type_name[e.cls] = orig_name
            continue
        # 충돌 — 각 entry 에 module prefix
        for e in entries:
            mod_pascal = pascal(_module_short(e.module))
            new_name = (
                orig_name if orig_name.startswith(mod_pascal) else f"{mod_pascal}{orig_name}"
            )
            if isinstance(e, BaseModelEntry):
                catalog.type_name[e.cls] = new_name
                e.name = new_name
            else:
                e.name = new_name


def _module_short(qualified: str) -> str:
    """`modules.motor.contract` → `motor`."""
    parts = qualified.split(".")
    return parts[-2] if len(parts) >= 2 else qualified


# ─── type → TS string ─────────────────────────────────────────────


def ts_type(ann: Any, catalog: Catalog) -> str:
    """Python annotation → TS type string. recursive."""
    if ann is type(None):
        return "null"
    if ann is int or ann is float:
        return "number"
    if ann is str:
        return "string"
    if ann is bool:
        return "boolean"
    if ann is bytes or ann is bytearray:
        return "Uint8Array"
    if ann is Any:
        return "unknown"

    origin = get_origin(ann)
    args = get_args(ann)

    if origin is Union or origin is _types.UnionType:
        parts = [ts_type(a, catalog) for a in args]
        return " | ".join(dict.fromkeys(parts))  # dedupe 순서 보존
    if origin is Literal:
        return " | ".join(_ts_literal(v) for v in args)
    if origin in (list, set, frozenset):
        inner = ts_type(args[0], catalog) if args else "unknown"
        return f"{inner}[]"
    if origin is tuple:
        if len(args) == 2 and args[1] is Ellipsis:
            return f"{ts_type(args[0], catalog)}[]"
        parts = [ts_type(a, catalog) for a in args]
        return "[" + ", ".join(parts) + "]"
    if origin is dict:
        if len(args) == 2:
            key_t = ts_type(args[0], catalog)
            val_t = ts_type(args[1], catalog)
            # TS Record key 는 string/number/symbol — 다른 자리는 string fallback
            if key_t not in ("string", "number"):
                key_t = "string"
            return f"Record<{key_t}, {val_t}>"
        return "Record<string, unknown>"

    # bare class
    if isinstance(ann, type):
        if issubclass(ann, BaseModel):
            return catalog.type_name.get(ann, ann.__name__)
        if issubclass(ann, Enum) and issubclass(ann, str):
            # StrEnum reference — value union type 통과 (`<Name>Value`)
            return f"{ann.__name__}Value"

    # fallback
    return "unknown"


def _ts_literal(v: Any) -> str:
    if isinstance(v, str):
        return _json_str(v)
    if isinstance(v, bool):
        return "true" if v else "false"
    if v is None:
        return "null"
    return repr(v)


def _json_str(s: str) -> str:
    """JSON-style double-quoted string. embedded `"` 와 `\\` 만 escape."""
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


# ─── BaseModel dependency graph + topological sort ────────────────


def topo_models(catalog: Catalog) -> list[BaseModelEntry]:
    """BaseModel field reference graph DFS topological sort."""
    visiting: set[type] = set()
    visited: set[type] = set()
    order: list[BaseModelEntry] = []
    by_cls = {e.cls: e for e in catalog.models.values()}

    def visit(cls: type[BaseModel]) -> None:
        if cls in visited or cls not in by_cls:
            return
        if cls in visiting:
            return  # cycle — 무시 (실 도메인 자리 cycle 없음)
        visiting.add(cls)
        for finfo in cls.model_fields.values():
            for dep in _collect_basemodel_deps(finfo.annotation):
                visit(dep)
        visiting.discard(cls)
        visited.add(cls)
        order.append(by_cls[cls])

    for cls in by_cls:
        visit(cls)
    return order


def _collect_basemodel_deps(ann: Any) -> list[type[BaseModel]]:
    out: list[type[BaseModel]] = []
    _walk(ann, out)
    return out


def _walk(ann: Any, out: list[type[BaseModel]]) -> None:
    if isinstance(ann, type):
        if issubclass(ann, BaseModel):
            out.append(ann)
        return
    for a in get_args(ann):
        _walk(a, out)


# ─── emit ────────────────────────────────────────────────────────


def emit(catalog: Catalog) -> str:
    lines: list[str] = []
    lines.append("/**")
    lines.append(" * Auto-generated by backend_v2/scripts/gen_contract.py")
    lines.append(" * Source: backend_v2 modules contract.py + impl.py")
    lines.append(" * DO NOT EDIT - run `pnpm gen:types` to regenerate.")
    lines.append(" */")
    lines.append("")

    # ─── StrEnum (const + value union) ────────────────────────
    for enum_entry in catalog.enums.values():
        name = enum_entry.name
        lines.append(f"export const {name} = {{")
        for mname, mvalue in enum_entry.members:
            lines.append(f"  {mname}: {_json_str(mvalue)},")
        lines.append("} as const;")
        lines.append(
            f"export type {name}Value = (typeof {name})[keyof typeof {name}];"
        )
        lines.append("")

    # ─── BaseModel interface (topological) ───────────────────
    for entry in topo_models(catalog):
        cls = entry.cls
        name = catalog.type_name.get(cls, cls.__name__)
        lines.append(f"export interface {name} {{")
        for fname, finfo in cls.model_fields.items():
            optional = (
                finfo.default is not PydanticUndefined
                or finfo.default_factory is not None
            )
            t = ts_type(finfo.annotation, catalog)
            sep = "?:" if optional else ":"
            lines.append(f"  {fname}{sep} {t};")
        lines.append("}")
        lines.append("")

    # ─── Topic (Stream + Event) + TopicPayloadMap ─────────────
    topic_entries = [k for k in catalog.keys if k.category in ("stream", "event")]
    topic_entries.sort(key=lambda k: k.const_name)

    lines.append("export const Topic = {")
    for k in topic_entries:
        lines.append(f"  {k.const_name}: {_json_str(k.key)},")
    lines.append("} as const;")
    lines.append("export type TopicKey = (typeof Topic)[keyof typeof Topic];")
    lines.append("")

    lines.append("export type TopicPayloadMap = {")
    for k in topic_entries:
        payload_t = (
            catalog.type_name.get(k.req_cls, k.req_cls.__name__)
            if k.req_cls is not None
            else "unknown"
        )
        lines.append(f"  {_json_str(k.key)}: {payload_t};")
    lines.append("};")
    lines.append("")

    # ─── ServiceKey + ServiceMap ──────────────────────────────
    service_entries = [k for k in catalog.keys if k.category == "service"]
    service_entries.sort(key=lambda k: k.const_name)

    lines.append("export const ServiceKey = {")
    for k in service_entries:
        lines.append(f"  {k.const_name}: {_json_str(k.key)},")
    lines.append("} as const;")
    lines.append(
        "export type ServiceKeyValue = (typeof ServiceKey)[keyof typeof ServiceKey];"
    )
    lines.append("")

    lines.append("export type ServiceMap = {")
    for k in service_entries:
        req_t = (
            catalog.type_name.get(k.req_cls, k.req_cls.__name__)
            if k.req_cls is not None
            else "unknown"
        )
        res_t = (
            catalog.type_name.get(k.res_cls, k.res_cls.__name__)
            if k.res_cls is not None
            else "unknown"
        )
        lines.append(
            f"  {_json_str(k.key)}: " "{ " f"req: {req_t}; res: {res_t}" " };"
        )
    lines.append("};")
    lines.append("")

    return "\n".join(lines)


# ─── main ────────────────────────────────────────────────────────


def build_catalog(modules_root: Path) -> Catalog:
    catalog = Catalog()
    keys_by_value: dict[str, KeyEntry] = {}

    mod_dirs = discover_module_dirs(modules_root)
    impl_mods: list[ModuleType] = []

    # pass 1 — load + collect enums/models/keys (no payload mapping yet)
    for mod_name in mod_dirs:
        contract_mod = load_contract(mod_name)
        mod_dir = modules_root / mod_name
        impl_mods.extend(load_module_py_files(mod_dir, mod_name))

        prefix = pascal(mod_name)
        outer = find_outer_class(contract_mod, prefix)

        # nested class 이름 — collect_enums 가 제외
        nested_names = {c for c in NESTED_CATEGORIES if outer is not None and hasattr(outer, c)}

        for ename, eentry in collect_enums(contract_mod, nested_names).items():
            catalog.enums[f"{mod_name}.{ename}"] = eentry
        for mname, mentry in collect_models(contract_mod).items():
            catalog.models[f"{mod_name}.{mname}"] = mentry
        for k in collect_keys_from_outer(outer, prefix):
            catalog.keys.append(k)
            keys_by_value[k.key] = k

    # pass 2 — payload mapping (impl file 들의 @publishes / @service / @subscriber spec)
    for impl_mod in impl_mods:
        fill_payload_from_module_py(keys_by_value, impl_mod)

    # name conflict 해소
    resolve_names(catalog)

    return catalog


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, help="output TS file path")
    parser.add_argument(
        "--modules-root",
        default="modules",
        help="backend_v2 modules/ root (default: ./modules)",
    )
    args = parser.parse_args(argv)

    root = Path.cwd()  # backend_v2/ 에서 실행 가정
    modules_root = (root / args.modules_root).resolve()
    if not modules_root.is_dir():
        print(f"modules root not found: {modules_root}", file=sys.stderr)
        return 2

    sys.path.insert(0, str(root))

    catalog = build_catalog(modules_root)
    ts = emit(catalog)

    out_path = Path(args.out).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(ts, encoding="utf-8", newline="\n")

    print(
        f"[gen_contract] {out_path} -- "
        f"enums={len(catalog.enums)} models={len(catalog.models)} "
        f"topics={sum(1 for k in catalog.keys if k.category != 'service')} "
        f"services={sum(1 for k in catalog.keys if k.category == 'service')}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
