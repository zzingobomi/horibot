"""SharedConfigModule — 공유 config owner (boundary = contract.py docstring).

workcell ROI 의 SSOT 흐름:
    boot: apps/config.py 가 instance.yaml 파싱 → resolve 가 초기값 주입
    read: SNAPSHOT_WORKCELL (Mirror snapshot — detector/frontend 패널)
    write: SET_WORKCELL → instance.yaml 반영(주석 보존) → WORKCELL_CHANGED
           publish → 소비자 Mirror 자동 수렴 (재시작 0)

쓰기 순서 계약: **yaml 영속 먼저, 성공해야 메모리/publish** — 파일 쓰기 실패가
"메모리만 바뀐 유령 상태"(재부팅 시 롤백되는 침묵 불일치)를 만들지 않는다.
실패는 RemoteError 로 전파 — frontend 는 draft 유지 + 사유 표시.
"""

from __future__ import annotations

import asyncio
import logging
import time
from pathlib import Path

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime

from .contract import (
    SetWorkcellRequest,
    SetWorkcellResponse,
    SharedConfig,
    SnapshotWorkcellRequest,
    WorkcellBundle,
    WorkcellChanged,
    WorkcellRoi,
)

logger = logging.getLogger(__name__)


def _write_workcell_yaml(instance_path: Path, roi: WorkcellRoi) -> None:
    """instance.yaml 의 `workcell:` 블록 갱신 — **손 주석/순서 보존** (ruamel
    round-trip). 파일이 없으면 workcell 만 담아 생성 (port 등은 손 소유 그대로
    비워둠). blocking I/O — 호출부가 to_thread 로 뺀다."""
    from ruamel.yaml import YAML  # lazy — 쓰기 경로에서만 필요

    yaml_rt = YAML()  # round-trip 모드 기본 — 주석·키 순서·따옴표 보존
    yaml_rt.preserve_quotes = True
    if instance_path.exists():
        data = yaml_rt.load(instance_path.read_text(encoding="utf-8")) or {}
    else:
        instance_path.parent.mkdir(parents=True, exist_ok=True)
        data = {}
    block = data.get("workcell")
    if block is None:
        block = {}
        data["workcell"] = block
    for key, value in roi.model_dump().items():
        block[key] = value
    with instance_path.open("w", encoding="utf-8") as f:
        yaml_rt.dump(data, f)


@publishes((SharedConfig.Event.WORKCELL_CHANGED, WorkcellChanged))
class SharedConfigModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        workcell: dict[str, WorkcellRoi] | None = None,
        instances_dir: Path | None = None,
    ) -> None:
        self.runtime = runtime
        # 초기값 = boot 시 config.py 가 파싱한 instance.yaml (resolve 주입).
        self._workcell: dict[str, WorkcellRoi] = dict(workcell or {})
        # 영속 루트 (robot/instances/) — None = 영속 불가 배포 (SET 명시 거부).
        self._instances_dir = instances_dir
        self._seq = 0
        self._write_lock = asyncio.Lock()  # SET 직렬화 (yaml 동시 쓰기 방지)

    async def start(self) -> None:
        logger.info(
            "SharedConfigModule start — workcell %d robot: %s",
            len(self._workcell), sorted(self._workcell),
        )

    async def stop(self) -> None:
        pass

    @service(SharedConfig.Service.SNAPSHOT_WORKCELL)
    async def snapshot_workcell(self, req: SnapshotWorkcellRequest) -> WorkcellBundle:
        return WorkcellBundle(robots=dict(self._workcell))

    @service(SharedConfig.Service.SET_WORKCELL)
    async def set_workcell(self, req: SetWorkcellRequest) -> SetWorkcellResponse:
        if self._instances_dir is None:
            raise RuntimeError(
                "이 배포는 workcell 영속 경로(instances_dir)가 없습니다 — "
                "instance.yaml 이 있는 host 에서 실행하세요"
            )
        path = self._instances_dir / req.robot_id / "instance.yaml"
        async with self._write_lock:
            # 영속 먼저 — 실패 시 메모리/publish 없이 그대로 전파 (module docstring)
            await asyncio.to_thread(_write_workcell_yaml, path, req.roi)
            self._workcell[req.robot_id] = req.roi
            self._seq += 1
            self.runtime.publish(
                SharedConfig.Event.WORKCELL_CHANGED,
                WorkcellChanged(
                    robot_id=req.robot_id,
                    seq=self._seq,
                    timestamp_unix=time.time(),
                    roi=req.roi,
                ),
            )
        logger.info(
            "workcell 저장 (%s): x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] → %s",
            req.robot_id, req.roi.x_min, req.roi.x_max, req.roi.y_min,
            req.roi.y_max, req.roi.z_min, req.roi.z_max, path,
        )
        return SetWorkcellResponse(roi=req.roi)
