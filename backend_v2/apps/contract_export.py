"""Contract serializer — running runtime 의 계약 → frontend contract JSON.

frontend_contract_gen.md §6.2 + §7. bridge 의 GET /contract.json 이 이걸로
직렬화한다. gen 스크립트(client)는 이 JSON 만 fetch → TS 조립 (backend import 0).

두 source 를 합친다:
  - **contract.py** (import-light: StrEnum + BaseModel 뿐) — model / enum / key /
    const 이름 / name-conflict 해소. serializer 가 직접 import (heavy dep 0).
  - **ContractSnapshot** (running runtime) — wire_key → payload 매핑. 이건 module.py
    핸들러 시그니처에만 있어 import 없이는 못 얻으니, 이미 import 를 끝낸 runtime 이
    내준다. **여기서 module.py 를 import 하지 않는 게 핵심** (§1 문제 해결).

노출 정책 = FRONTEND_EXPOSED (opt-in allowlist) — 여기 한 곳. contract.py /
module.py 는 프론트 노출 개념을 모른 채 순수하게 유지된다 (§5 제약 1·2·3).

옛 backend_v2/scripts/gen_contract.py 의 catalog/ts_type/reachability/resolve_names
로직을 그대로 옮긴 것 — 달라진 건 payload 를 module.py import 대신 snapshot 에서
채운다는 점 (fill_payload_from_snapshot) + emit 이 TS 문자열 대신 JSON dict 를
낸다는 점 (조립은 gen 스크립트로 이관, §6.4).
"""

from __future__ import annotations

import importlib
import inspect
import types as _types
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from types import ModuleType
from typing import Any, Literal, Union, get_args, get_origin

from pydantic import BaseModel
from pydantic_core import PydanticUndefined

from framework.runtime.snapshot import (
    ContractSnapshot,
    ModuleContract,
    build_module_contracts_from_classes,
    build_snapshot_from_classes,
)
from modules.bridge.contract import RobotsResponse, SystemMetrics
from modules.calibration.contract import Calibration
from modules.motion.contract import Motion
from modules.motor.contract import Motor

_MODULES_ROOT = Path(__file__).resolve().parents[1] / "modules"


# ─── frontend 노출 manifest (opt-in allowlist) ─────────────────────
#
# 왜 여기 있나: contract.py = 백엔드 module 간 Zenoh 계약(SSOT). 프론트는 bridge
# (투명 relay)를 통해 그 subset 만 소비. "무엇을 노출할지" 는 백엔드 계약의 성격이
# 아니라 프론트 계약 생성의 관심사 → serializer 가 그 목록의 집. contract.py /
# module.py 는 이 개념을 모른 채 순수하게 유지.
#
# 왜 allowlist 인가: "쓰이니까 노출"(reachability)은 틀린 축 — Motor.Stream.COMMAND
# (Motion→Motor 내부 wire) 는 module 간엔 쓰이지만 프론트엔 절대 노출 X. "공개
# API 인가" 는 사용 여부와 독립.
#
# opt-in: 백엔드 개발 시엔 노출 고민 X. 나중에 "프론트가 명령/구독해야겠다" 결정
# 순간 여기 한 줄 추가. 안 적으면 미노출(누출 아님). enum 멤버 직접 참조 —
# rename/삭제 시 import 에서 즉시 터짐.
FRONTEND_EXPOSED: set[str] = {
    str(k)
    for k in (
        Motor.Service.CAPABILITIES,
        Motor.Service.GET_TOPOLOGY,
        Motor.Service.SET_TORQUE,
        Motor.Stream.RAW_STATE,
        Motor.Stream.STATE,
        Motor.Event.TORQUE_CHANGED,
        Motion.Service.MOVE_J,
        Motion.Stream.TCP_STATE,
        Motion.Stream.JOG_J,
        Motion.Stream.JOG_TCP,
        # calibration — RobotCalibrateMode 페이지
        Calibration.Service.START_RUN,
        Calibration.Service.CAPTURE,
        Calibration.Service.UNDO_LAST_CAPTURE,
        Calibration.Service.FINALIZE_RUN,
        Calibration.Service.ACTIVATE_RESULT,
        Calibration.Service.PREVIEW_ENABLE,
        Calibration.Service.SNAPSHOT_BUNDLE,
        Calibration.Service.LIST_RUNS,
        Calibration.Service.LIST_RESULTS,
        Calibration.Service.GET_THRESHOLDS,
        Calibration.Stream.PREVIEW,
        Calibration.Event.ACTIVATED,
        Calibration.Event.COMMITTED,
    )
}

# HTTP resource 모델 — Zenoh 키 뒤에 없고 bridge HTTP endpoint (/robots, /system)
# 로 나가는 프론트 계약. 키 reachability 로 안 잡히므로 별도 seed.
# (RobotInfo / BasePoseInfo 는 RobotsResponse 에서 도달하므로 안 적어도 됨.)
FRONTEND_EXPOSED_MODELS: set[type] = {RobotsResponse, SystemMetrics}


def check_exposed(all_keys: set[str], exposed: set[str]) -> None:
    """FRONTEND_EXPOSED 의 모든 키가 실제 discovered(contract.py enum) 키인지 검증.

    opt-in 이라 "미분류" 는 문제 아님. 하지만 EXPOSED 에 존재하지 않는 키(오타 /
    삭제됐는데 남은 것)가 있으면 fail-fast."""
    stale = exposed - all_keys
    if stale:
        raise ValueError(
            f"[contract_export] FRONTEND_EXPOSED 에 discovered 되지 않은 키: "
            f"{sorted(stale)}"
        )


# ─── module discovery ──────────────────────────────────────────────

NESTED_CATEGORIES = ("Service", "Stream", "Event")


def pascal(name: str) -> str:
    return "".join(p.capitalize() for p in name.split("_"))


def discover_module_dirs(modules_root: Path) -> list[str]:
    out: list[str] = []
    for child in sorted(modules_root.iterdir()):
        if child.is_dir() and (child / "contract.py").is_file():
            out.append(child.name)
    return out


# ─── catalog dataclass ─────────────────────────────────────────────


@dataclass
class StrEnumEntry:
    name: str
    module: str
    members: list[tuple[str, str]]
    cls: type


@dataclass
class BaseModelEntry:
    name: str
    module: str
    cls: type[BaseModel]


@dataclass
class KeyEntry:
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
    type_name: dict[type, str] = field(default_factory=dict)


# ─── load + extract (contract.py 만 import — import-light) ─────────


def load_contract(mod_name: str) -> ModuleType:
    return importlib.import_module(f"modules.{mod_name}.contract")


def find_outer_class(contract_mod: ModuleType, prefix: str) -> type | None:
    obj = getattr(contract_mod, prefix, None)
    if isinstance(obj, type) and not issubclass(obj, BaseModel):
        return obj
    return None


def collect_enums(
    contract_mod: ModuleType, exclude: set[str]
) -> dict[str, StrEnumEntry]:
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
        out[name] = StrEnumEntry(
            name=name, module=contract_mod.__name__, members=members, cls=obj
        )
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
    """Service/Stream/Event nested StrEnum → KeyEntry (payload 는 snapshot 이 채움)."""
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


def fill_payload_from_snapshot(catalog: Catalog, snapshot: ContractSnapshot) -> None:
    """running runtime 의 wire_key → payload 매핑을 catalog key 에 주입.

    옛 gen 의 fill_payload_from_module_py(module.py import) 를 대체 — heavy dep 회피
    의 핵심. snapshot 이 이미 dedup / 충돌 검증했으니 여기선 단순 대입.
    enum 에 없는 wire_key (raw string 등) 는 frontend 계약 대상 아니라 skip."""
    by_key = {k.key: k for k in catalog.keys}
    for wire_key, (req_cls, res_cls) in snapshot.services.items():
        entry = by_key.get(wire_key)
        if entry is not None:
            entry.req_cls = req_cls
            entry.res_cls = res_cls
    for wire_key, payload_cls in snapshot.topics.items():
        entry = by_key.get(wire_key)
        if entry is not None:
            entry.req_cls = payload_cls


# ─── name conflict resolution ─────────────────────────────────────


def resolve_names(catalog: Catalog) -> None:
    """충돌 자리만 module prefix. 중복 없으면 원본 이름.

    주의: 충돌 판정은 *모든 module 의 contract.py* 전체 model 집합 기준 —
    노출 subset 아님. 예: motor.CapabilitiesRequest 가 MotorCapabilitiesRequest 로
    prefix 되는 건 camera.CapabilitiesRequest 가 존재하기 때문. 그래서 serializer 는
    노출 안 하는 module 의 contract.py 도 import 해서 universe 를 완성한다."""
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
        for e in entries:
            mod_pascal = pascal(_module_short(e.module))
            new_name = (
                orig_name
                if orig_name.startswith(mod_pascal)
                else f"{mod_pascal}{orig_name}"
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
            if key_t not in ("string", "number"):
                key_t = "string"
            return f"Record<{key_t}, {val_t}>"
        return "Record<string, unknown>"

    if isinstance(ann, type):
        if issubclass(ann, BaseModel):
            return catalog.type_name.get(ann, ann.__name__)
        if issubclass(ann, Enum) and issubclass(ann, str):
            return f"{ann.__name__}Value"

    return "unknown"


def _ts_literal(v: Any) -> str:
    if isinstance(v, str):
        return '"' + v.replace("\\", "\\\\").replace('"', '\\"') + '"'
    if isinstance(v, bool):
        return "true" if v else "false"
    if v is None:
        return "null"
    return repr(v)


# ─── BaseModel dependency graph + topological sort ────────────────


def topo_models(catalog: Catalog) -> list[BaseModelEntry]:
    visiting: set[type] = set()
    visited: set[type] = set()
    order: list[BaseModelEntry] = []
    by_cls = {e.cls: e for e in catalog.models.values()}

    def visit(cls: type[BaseModel]) -> None:
        if cls in visited or cls not in by_cls:
            return
        if cls in visiting:
            return  # cycle — 무시 (실 도메인 cycle 없음)
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


# ─── frontend reachability (노출 키 → 도달 model / enum) ───────────


def _collect_enum_refs(ann: Any, out: set[type]) -> None:
    if isinstance(ann, type):
        if issubclass(ann, Enum) and issubclass(ann, str) and ann is not str:
            out.add(ann)
        return
    for a in get_args(ann):
        _collect_enum_refs(a, out)


def reachable_models(
    exposed_keys: list[KeyEntry],
    catalog: Catalog,
    extra_seeds: set[type] = frozenset(),  # type: ignore[assignment]
) -> set[type]:
    """노출 키의 req/res/payload (+extra_seeds) 에서 field 그래프로 도달 가능한
    BaseModel 집합. 내부 전용 payload 는 도달 안 되면 emit 에서 자동 제외."""
    by_cls = {e.cls for e in catalog.models.values()}
    seen: set[type] = set()
    stack: list[type] = list(extra_seeds)
    for k in exposed_keys:
        for c in (k.req_cls, k.res_cls):
            if c is not None:
                stack.append(c)
    while stack:
        cls = stack.pop()
        if cls in seen or cls not in by_cls:
            continue
        seen.add(cls)
        for finfo in cls.model_fields.values():
            for dep in _collect_basemodel_deps(finfo.annotation):
                stack.append(dep)
    return seen


def referenced_enums(reachable: set[type]) -> set[type]:
    out: set[type] = set()
    for cls in reachable:
        for finfo in cls.model_fields.values():
            _collect_enum_refs(finfo.annotation, out)
    return out


# ─── catalog build (contract.py only — no module.py) ──────────────


def build_catalog(modules_root: Path = _MODULES_ROOT) -> Catalog:
    catalog = Catalog()
    for mod_name in discover_module_dirs(modules_root):
        contract_mod = load_contract(mod_name)
        prefix = pascal(mod_name)
        outer = find_outer_class(contract_mod, prefix)
        nested_names = {
            c for c in NESTED_CATEGORIES if outer is not None and hasattr(outer, c)
        }
        for ename, eentry in collect_enums(contract_mod, nested_names).items():
            catalog.enums[f"{mod_name}.{ename}"] = eentry
        for mname, mentry in collect_models(contract_mod).items():
            catalog.models[f"{mod_name}.{mname}"] = mentry
        for k in collect_keys_from_outer(outer, prefix):
            catalog.keys.append(k)
    resolve_names(catalog)
    return catalog


# ─── build contract JSON (bridge GET /contract.json 응답) ─────────


def build_contract_json(
    snapshot: ContractSnapshot, modules_root: Path = _MODULES_ROOT
) -> dict:
    """running runtime snapshot + contract.py → frontend contract JSON.

    gen 스크립트가 이 JSON 을 fetch → contract.ts 조립. type 은 이미 TS 문자열로
    해소돼 있어 (서버가 실 Python type 을 아니까) gen 은 순수 문자열 조립만."""
    catalog = build_catalog(modules_root)
    fill_payload_from_snapshot(catalog, snapshot)

    all_keys = {k.key for k in catalog.keys}
    check_exposed(all_keys, FRONTEND_EXPOSED)

    exposed_keys = [k for k in catalog.keys if k.key in FRONTEND_EXPOSED]

    # incomplete-host guard — 노출 키인데 running runtime 에 payload 가 없음 =
    # 그 module 이 이 host 에 로드 안 됨. gen 은 전 module 로드하는 mock/dev 로.
    missing = sorted(k.key for k in exposed_keys if k.req_cls is None)
    if missing:
        raise RuntimeError(
            "다음 FRONTEND_EXPOSED 키의 payload 가 running runtime 에 없음 — 해당 "
            "module 이 이 host 에 로드 안 됨. gen 은 전 module 로드하는 mock/dev "
            f"host 로 실행해야 함: {missing}"
        )

    reachable = reachable_models(exposed_keys, catalog, FRONTEND_EXPOSED_MODELS)
    ref_enums = referenced_enums(reachable)

    enums_out: list[dict] = []
    for e in catalog.enums.values():
        if e.cls not in ref_enums:
            continue
        enums_out.append(
            {"name": e.name, "members": [[n, v] for n, v in e.members]}
        )

    interfaces_out: list[dict] = []
    for entry in topo_models(catalog):
        cls = entry.cls
        if cls not in reachable:
            continue
        name = catalog.type_name.get(cls, cls.__name__)
        fields: list[dict] = []
        for fname, finfo in cls.model_fields.items():
            optional = (
                finfo.default is not PydanticUndefined
                or finfo.default_factory is not None
            )
            fields.append(
                {
                    "name": fname,
                    "ts": ts_type(finfo.annotation, catalog),
                    "optional": optional,
                }
            )
        interfaces_out.append({"name": name, "fields": fields})

    topic_keys = sorted(
        (k for k in exposed_keys if k.category in ("stream", "event")),
        key=lambda k: k.const_name,
    )
    topics_out = [
        {
            "const": k.const_name,
            "key": k.key,
            "payload": _type_name(catalog, k.req_cls),
        }
        for k in topic_keys
    ]

    service_keys = sorted(
        (k for k in exposed_keys if k.category == "service"),
        key=lambda k: k.const_name,
    )
    services_out = [
        {
            "const": k.const_name,
            "key": k.key,
            "req": _type_name(catalog, k.req_cls),
            "res": _type_name(catalog, k.res_cls),
        }
        for k in service_keys
    ]

    return {
        "enums": enums_out,
        "interfaces": interfaces_out,
        "topics": topics_out,
        "services": services_out,
    }


def _type_name(catalog: Catalog, cls: type[BaseModel] | None) -> str:
    if cls is None:
        return "unknown"
    return catalog.type_name.get(cls, cls.__name__)


# ─── contract graph (dev viewer — contract_graph_viewer.md §5.2) ──────
#
# build_contract_json 과 대칭: 같은 catalog / snapshot / ts_type / name-conflict
# 재사용, 다른 축 3개 —
#   1. **unfiltered** — FRONTEND_EXPOSED subset 이 아니라 전 module 의 전 계약
#      (개발자 가시성 목적).
#   2. **attribution 보존** — 어느 module 이 무엇을 serve/publish/subscribe 하는지
#      (build_contract_json 은 wire_key→payload flat, 이건 module→wire 방향).
#   3. **그래프 구조** — stream/event 는 publisher module → subscriber module 방향
#      엣지. service 는 caller 를 정적으로 못 잡으니 owner node 속성으로만 (§2 한계).
#
# backend 는 React Flow 형식을 모르는 **중립 그래프** (position/layout 은 frontend).

_CATEGORY_BY_PREFIX = {"srv": "service", "stream": "stream", "event": "event"}


def _key_category(wire_key: str) -> str:
    return _CATEGORY_BY_PREFIX.get(wire_key.split("/", 1)[0], "unknown")


def _key_domain(wire_key: str) -> str:
    """wire_key `srv/motor/{robot_id}/x` → domain `motor` (그룹 색상 등)."""
    parts = wire_key.split("/")
    return parts[1] if len(parts) >= 2 else ""


def _module_domain(mc: ModuleContract) -> str:
    for group in (mc.services, mc.publishes, mc.subscribes):
        for k in group:
            dom = _key_domain(k)
            if dom:
                return dom
    return ""


def build_contract_graph(
    module_contracts: list[ModuleContract],
    snapshot: ContractSnapshot,
    modules_root: Path = _MODULES_ROOT,
) -> dict:
    """running runtime 의 module attribution + snapshot → 중립 계약 그래프 JSON.

    응답 스키마 = contract_graph_viewer.md §4: {modules, keys, models, edges}.
    필터 없음 (전 계약). name-conflict 해소는 build_contract_json 과 동일 catalog
    (전 module contract.py universe) 로 — 그래야 keys 가 참조하는 payload 이름이
    models dict key 와 정확히 매칭 (CapabilitiesRequest 충돌 → 접두사)."""
    catalog = build_catalog(modules_root)
    fill_payload_from_snapshot(catalog, snapshot)
    by_key = {k.key: k for k in catalog.keys}

    # contract 하나도 없는 Module (Bridge 같은 relay) 는 wiring 에 기여 0 → node 제외.
    contentful = [
        mc
        for mc in module_contracts
        if mc.services or mc.publishes or mc.subscribes
    ]

    modules_out = [
        {
            "id": mc.module_id,
            "domain": _module_domain(mc),
            "robot_scoped": mc.robot_scoped,
            "services": list(mc.services),
            "publishes": list(mc.publishes),
            "subscribes": list(mc.subscribes),
        }
        for mc in contentful
    ]

    # keys — module 들이 참조하는 전 wire_key 의 category + payload/req/res 이름.
    referenced: set[str] = set()
    for mc in contentful:
        referenced |= {*mc.services, *mc.publishes, *mc.subscribes}
    keys_out: dict[str, dict] = {}
    for wk in sorted(referenced):
        entry = by_key.get(wk)
        if entry is None:
            # contract.py enum 에 없는 raw key — frontend gen 과 동일하게 skip
            continue
        if entry.category == "service":
            keys_out[wk] = {
                "category": "service",
                "req": _type_name(catalog, entry.req_cls),
                "res": _type_name(catalog, entry.res_cls),
            }
        else:
            keys_out[wk] = {
                "category": entry.category,
                # topic payload 는 fill_payload_from_snapshot 이 req_cls 에 저장
                "payload": _type_name(catalog, entry.req_cls),
            }

    # models — 참조된 전 payload/req/res 에서 field 그래프로 도달 가능한 model 의
    # field:type map (드릴다운용). unfiltered — 노출 subset 아님.
    reachable = reachable_models(catalog.keys, catalog)
    models_out: dict[str, dict[str, str]] = {}
    for entry in topo_models(catalog):
        cls = entry.cls
        if cls not in reachable:
            continue
        name = catalog.type_name.get(cls, cls.__name__)
        models_out[name] = {
            fname: ts_type(finfo.annotation, catalog)
            for fname, finfo in cls.model_fields.items()
        }

    # edges — stream/event key 별 publisher module × subscriber module 곱 (방향).
    # service 엣지 없음 (§2 caller 정적으로 못 잡음 — owner node 속성으로만).
    publishers: dict[str, list[str]] = {}
    subscribers: dict[str, list[str]] = {}
    for mc in contentful:
        for wk in mc.publishes:
            publishers.setdefault(wk, []).append(mc.module_id)
        for wk in mc.subscribes:
            subscribers.setdefault(wk, []).append(mc.module_id)

    edges_out: list[dict] = []
    for wk in sorted(set(publishers) | set(subscribers)):
        category = _key_category(wk)
        for src in publishers.get(wk, []):
            for dst in subscribers.get(wk, []):
                edges_out.append(
                    {
                        "source": src,
                        "target": dst,
                        "key": wk,
                        "category": category,
                    }
                )

    return {
        "modules": modules_out,
        "keys": keys_out,
        "models": models_out,
        "edges": edges_out,
    }


# ─── static graph — 분산 배치 자리 declared universe 그림 (개발자 뷰어) ─
#
# 문제: bridge 프로세스는 자기 runtime 의 module_contracts() 만 봄. 분산 배치
# (PC = camera_decoded + bridge, pi_motor = motor + motion) 자리 PC bridge 의
# `/contract/graph` 는 자기 프로세스에 로드된 module 만 그린다 → 개발자 뷰어
# 목적 (전 fleet 아키텍처 한눈에) 위반.
#
# 해결: bridge 는 자기 runtime 대신 `MODULE_REGISTRY` (선언된 전 module) 를
# lazy import 해서 introspect. class 자체는 @service / @subscriber / @publishes
# spec 을 attribute 로 들고 있어 instantiate 없이 decorator spec 추출 가능.
# contract_graph_viewer.md §1: "개발 도구", §4: "unfiltered 전 module 의 전 계약".


def build_static_contract_graph() -> dict:
    """MODULE_REGISTRY 전체 → contract graph. running fleet 무관, declared universe."""
    from apps.registry import MODULE_REGISTRY, load_module_class

    classes = [load_module_class(name) for name in MODULE_REGISTRY]
    module_contracts = build_module_contracts_from_classes(classes)
    snapshot = build_snapshot_from_classes(classes)
    return build_contract_graph(module_contracts, snapshot)
