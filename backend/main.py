import argparse
import logging
import signal
import socket
import sys
import threading
from pathlib import Path
from typing import Any

import yaml

from core.transport.application_node import ApplicationNode
from core.transport.device_node import DeviceNode
from core.transport.node_registry import (
    create_node,
    get_class,
    known_nodes,
)
from core.transport.zenoh_session import ZenohSession


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s - %(message)s",
)
logger = logging.getLogger(__name__)

BRIDGE_HOST = "0.0.0.0"
BRIDGE_PORT = 8000

CONFIG_DIR = Path(__file__).parent / "config"


def _resolve_config_path(host_arg: str | None) -> Path:
    if host_arg:
        path = CONFIG_DIR / f"host_{host_arg}.yaml"
        if not path.exists():
            raise FileNotFoundError(f"호스트 config 없음: {path}")
        return path

    hostname = socket.gethostname().lower().replace("-", "_")
    candidate = CONFIG_DIR / f"host_{hostname}.yaml"
    if candidate.exists():
        logger.info("hostname 매칭: %s", candidate.name)
        return candidate

    fallback = CONFIG_DIR / "host_dev.yaml"
    if not fallback.exists():
        raise FileNotFoundError(
            f"기본 config({fallback}) 없음. --host 인자로 명시 필요."
        )
    logger.info(
        "기본 모드(host_dev.yaml) 사용. 분산 배치는 --host pc/pi_motor/pi_camera로 명시."
    )
    return fallback


def _load_config(path: Path) -> dict[str, Any]:
    logger.info("호스트 config 로드: %s", path)
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser(description="Horibot 백엔드")
    parser.add_argument(
        "--host",
        default=None,
        help=(
            "호스트 이름 (config/host_<name>.yaml 사용). "
            "미지정 시 hostname 자동 감지 → 매칭 실패 시 host_dev.yaml(단일 머신)."
        ),
    )
    args = parser.parse_args()

    cfg = _load_config(_resolve_config_path(args.host))
    host_name = cfg.get("host_name", "?")
    robots: list[str] = list(cfg.get("robots", []))
    device_node_names: list[str] = list(cfg.get("device_nodes", []))
    application_node_names: list[str] = list(cfg.get("application_nodes", []))
    bridge_cfg: dict[str, Any] = cfg.get("bridge", {}) or {}

    logger.info("=== Horibot 시작 (host=%s) ===", host_name)
    logger.info(
        "robots=%s  device_nodes=%s  application_nodes=%s",
        robots, device_node_names, application_node_names,
    )

    # ─── 검증: robots.yaml 존재 + layer 위치 ───────────────────
    from core.robot.robot_registry import RobotRegistry

    registry = RobotRegistry()
    for rid in robots:
        if rid not in registry.list_robots():
            raise ValueError(
                f"host config 의 robots 에 있는 '{rid}' 가 robots.yaml 에 없음. "
                f"등록된: {registry.list_robots()}"
            )

    for name in device_node_names:
        if name not in known_nodes():
            raise ValueError(
                f"알 수 없는 노드 '{name}'. 등록: {known_nodes()}"
            )
        if not issubclass(get_class(name), DeviceNode):
            raise ValueError(
                f"'{name}' 은 DeviceNode 아님. application_nodes 로 옮겨야 함."
            )

    for name in application_node_names:
        if name not in known_nodes():
            raise ValueError(
                f"알 수 없는 노드 '{name}'. 등록: {known_nodes()}"
            )
        if not issubclass(get_class(name), ApplicationNode):
            raise ValueError(
                f"'{name}' 은 ApplicationNode 아님. device_nodes 로 옮겨야 함."
            )

    # device_nodes 가 있는데 robots 가 비어있으면 인스턴스 0개 — 잘못된 config
    if device_node_names and not robots:
        raise ValueError(
            "host config 의 device_nodes 가 있는데 robots 가 비어있음. "
            "device 노드 띄우려면 robots 명시 필요."
        )

    # ─── Zenoh 세션 초기화 ────────────────────────────────────
    ZenohSession.init(cfg.get("zenoh"))

    # ─── Storage 초기화 (storage 노드 떠 있을 때만) ────────────
    # PC 만 storage_node 띄움 — 분산 모드 모터/카메라 Pi 는 init() 호출 안 됨.
    # 단, storage 가 application_nodes 에 있는데 host yaml 의 'storage:' block
    # 누락이면 fail-fast.
    if "storage" in application_node_names:
        storage_cfg = cfg.get("storage") or {}
        rdb_uri = storage_cfg.get("rdb_uri")
        object_uri = storage_cfg.get("object_uri")
        if not rdb_uri or not object_uri:
            raise ValueError(
                "host config 의 application_nodes 에 'storage' 가 있는데 "
                "'storage:' block 의 rdb_uri / object_uri 누락. "
                "docs/storage_layer.md §8 참조."
            )
        from modules.storage.registry import StorageRegistry

        StorageRegistry.init(rdb_uri, object_uri)

    # ─── D405 intrinsic seed (camera 노드 robot 마다) ──────────
    # storage 의 active intrinsic 없을 때만 D405 SDK 의 factory intrinsic 을
    # commit + activate. idempotent — N회 부팅 = commit 최대 1회.
    if "camera" in device_node_names:
        from modules.camera.factory_intrinsic import seed_d405_intrinsic_to_storage
        for rid in robots:
            seed_d405_intrinsic_to_storage(robot_id=rid)

    # ─── 노드 인스턴스 생성 ───────────────────────────────────
    # Application 먼저, Device 다음 — main loop 가 dict iteration 순으로 start()
    # 호출하는데, motion_node.start() 가 storage service (joint/link/sag offsets
    # fetch_active) 무한 retry 로 blocking. application (storage) 가 같은
    # process 안에 있으면 device 보다 *먼저 start* 해야 retry 가 즉시 풀림.
    # 분산 자리 (모터 Pi 의 motion_node, PC 의 storage_node) 는 자기 process
    # 안에 storage 없음 → 무관, 별 process 의 storage 가 떠 있으면 retry 풀림.
    instances: dict[tuple[str, str | None], Any] = {}
    for name in application_node_names:
        instances[(name, None)] = create_node(name)
    for name in device_node_names:
        for rid in robots:
            instances[(name, rid)] = create_node(name, robot_id=rid)

    # ─── 노드 시작 ────────────────────────────────────────────
    # storage_layer.md §7 — 부팅 순서 강제 X. Storage 의존 노드 (Coordinates
    # 사용하는 device 등) 는 storage 가 늦게 떠도 retry loop 으로 자동 연결.
    for (nt, rid), node in instances.items():
        node.start()
        if rid is None:
            logger.info("노드 시작됨: %s (application, %s)", nt, node.node_name)
        else:
            logger.info("노드 시작됨: %s (device robot=%s, %s)", nt, rid, node.node_name)

    # ─── 브릿지 (선택) ────────────────────────────────────────
    bridge_enabled = bool(bridge_cfg.get("enabled", False))
    bridge_app = None
    if bridge_enabled:
        from bridge.zenoh_bridge import app as bridge_app
        from bridge.zenoh_bridge import setup_zenoh_subscribers

        setup_zenoh_subscribers()

    # ─── 종료 시그널 ──────────────────────────────────────────
    stop_event = threading.Event()

    def shutdown(_sig, _frame):
        logger.info("종료 신호 수신, 노드 정리 중...")
        for node in instances.values():
            try:
                node.stop()
            except Exception as e:
                logger.warning("노드 stop 중 오류 (%s): %s", node.node_name, e)
        ZenohSession.close()
        stop_event.set()
        # uvicorn이 띄워져 있으면 sys.exit가 깔끔하게 안 빠질 수 있어서 force exit.
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, shutdown)

    # ─── 메인 루프 ────────────────────────────────────────────
    if bridge_enabled and bridge_app is not None:
        import uvicorn

        host = bridge_cfg.get("host", "0.0.0.0")
        port = int(bridge_cfg.get("port", 8000))
        logger.info("브릿지 서버 시작: ws://%s:%s", host, port)
        uvicorn.run(bridge_app, host=host, port=port, log_level="warning")
    else:
        logger.info("브릿지 비활성화. 종료 신호 대기 중.")
        stop_event.wait()


if __name__ == "__main__":
    main()
