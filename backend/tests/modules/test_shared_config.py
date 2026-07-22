"""SharedConfigModule 테스트 — 공유 config owner (workcell ROI 첫 멤버).

의미 (뒤집으면 회귀): SET 이 instance.yaml 손 주석을 죽이고 재직렬화 /
영속 실패인데 메모리·publish 가 진행 (재부팅 롤백되는 유령 상태) /
min≥max ROI 가 침묵 통과 / SNAPSHOT 이 저장본과 불일치.
Mirror 수렴 자체(늦은 owner 부팅 포함)는 tests/framework/test_mirror.py 잠금.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml as pyyaml
from pydantic import BaseModel, ValidationError

from modules.shared_config.contract import (
    SetWorkcellRequest,
    SharedConfig,
    SnapshotWorkcellRequest,
    WorkcellRoi,
)
from modules.shared_config.module import SharedConfigModule

_BOT = "so101_6dof_0"

_ROI = WorkcellRoi(
    x_min=0.0, x_max=0.35, y_min=-0.30, y_max=0.30, z_min=-0.05, z_max=0.35
)
_ROI2 = WorkcellRoi(
    x_min=0.05, x_max=0.40, y_min=-0.25, y_max=0.25, z_min=-0.04, z_max=0.30
)

_INSTANCE_YAML = """\
# 손으로 쓴 주석 — SET 후에도 살아 있어야 한다 (ruamel round-trip 계약)
motor:
  port:
    windows: COM6  # port 주석
  baudrate: 1000000

# workcell 블록 주석
workcell:
  x_min: 0.0
  x_max: 0.35
  y_min: -0.30
  y_max: 0.30
  z_min: -0.05
  z_max: 0.35
"""


class _Rt:
    def __init__(self) -> None:
        self.published: list[tuple[str, BaseModel]] = []

    def publish(self, key, event) -> None:  # noqa: ANN001
        self.published.append((str(key), event))


def _module(tmp_path: Path, *, seed: bool = True) -> tuple[SharedConfigModule, _Rt]:
    inst = tmp_path / "instances" / _BOT
    inst.mkdir(parents=True)
    (inst / "instance.yaml").write_text(_INSTANCE_YAML, encoding="utf-8")
    rt = _Rt()
    mod = SharedConfigModule(
        rt,  # type: ignore[arg-type]
        workcell={_BOT: _ROI} if seed else {},
        instances_dir=tmp_path / "instances",
    )
    return mod, rt


async def test_snapshot_returns_seeded_bundle(tmp_path: Path):
    mod, _ = _module(tmp_path)
    bundle = await mod.snapshot_workcell(SnapshotWorkcellRequest())
    assert bundle.robots == {_BOT: _ROI}


async def test_set_persists_yaml_preserving_comments_and_publishes(tmp_path: Path):
    """SET = yaml 반영(손 주석·타 블록 보존) + 메모리 + CHANGED publish.
    주석 소멸은 pyyaml 재직렬화 회귀 (ruamel round-trip 이 계약)."""
    mod, rt = _module(tmp_path)
    res = await mod.set_workcell(SetWorkcellRequest(robot_id=_BOT, roi=_ROI2))
    assert res.roi == _ROI2

    text = (tmp_path / "instances" / _BOT / "instance.yaml").read_text("utf-8")
    assert "손으로 쓴 주석" in text and "port 주석" in text  # 주석 생존
    data = pyyaml.safe_load(text)
    assert data["motor"]["port"]["windows"] == "COM6"  # 타 블록 보존
    assert data["workcell"]["x_min"] == pytest.approx(0.05)
    assert data["workcell"]["z_max"] == pytest.approx(0.30)

    # 메모리(다음 snapshot) + publish 일치
    bundle = await mod.snapshot_workcell(SnapshotWorkcellRequest())
    assert bundle.robots[_BOT] == _ROI2
    assert len(rt.published) == 1
    key, event = rt.published[0]
    assert key == str(SharedConfig.Event.WORKCELL_CHANGED)
    assert event.robot_id == _BOT and event.roi == _ROI2  # type: ignore[attr-defined]


async def test_set_creates_instance_yaml_when_missing(tmp_path: Path):
    """instance.yaml 없는 robot (신규/instance 폴더 미생성) — 생성해 저장.
    저장이 '폴더 먼저 만들어 오세요' 로 튕기면 패널 UX 반쪽."""
    rt = _Rt()
    mod = SharedConfigModule(
        rt, workcell={}, instances_dir=tmp_path / "instances",  # type: ignore[arg-type]
    )
    await mod.set_workcell(SetWorkcellRequest(robot_id="new_bot_0", roi=_ROI))
    data = pyyaml.safe_load(
        (tmp_path / "instances" / "new_bot_0" / "instance.yaml").read_text("utf-8")
    )
    assert data["workcell"]["x_max"] == pytest.approx(0.35)


async def test_set_write_failure_keeps_memory_and_no_publish(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """영속 실패 = 그대로 전파 + 메모리/publish 없음 — 파일은 옛값인데 소비자만
    새값인 유령 상태(재부팅 롤백) 금지 (module docstring 쓰기 순서 계약)."""
    mod, rt = _module(tmp_path)

    def boom(path, roi) -> None:  # noqa: ANN001
        raise OSError("disk full")

    import modules.shared_config.module as sc_mod

    monkeypatch.setattr(sc_mod, "_write_workcell_yaml", boom)
    with pytest.raises(OSError, match="disk full"):
        await mod.set_workcell(SetWorkcellRequest(robot_id=_BOT, roi=_ROI2))
    bundle = await mod.snapshot_workcell(SnapshotWorkcellRequest())
    assert bundle.robots[_BOT] == _ROI  # 옛값 유지
    assert rt.published == []


async def test_set_without_instances_dir_rejected(tmp_path: Path):
    rt = _Rt()
    mod = SharedConfigModule(rt, workcell={_BOT: _ROI}, instances_dir=None)  # type: ignore[arg-type]
    with pytest.raises(RuntimeError, match="영속 경로"):
        await mod.set_workcell(SetWorkcellRequest(robot_id=_BOT, roi=_ROI2))


def test_roi_bounds_validation():
    """min ≥ max ROI 는 wire 진입 자체가 거부 — 패널 오입력이 침묵 저장돼
    detector 컷이 전 후보를 죽이는 사고 방지 (fail-fast)."""
    with pytest.raises(ValidationError, match="x_min"):
        WorkcellRoi(
            x_min=0.4, x_max=0.35, y_min=-0.3, y_max=0.3, z_min=0.0, z_max=0.3
        )
    with pytest.raises(ValidationError, match="z_min"):
        WorkcellRoi(
            x_min=0.0, x_max=0.35, y_min=-0.3, y_max=0.3, z_min=0.3, z_max=0.3
        )
