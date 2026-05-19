import argparse
import logging
import signal
import socket
import sys
import threading
from pathlib import Path
from typing import Any

import yaml

from core.zenoh_session import ZenohSession
from core.node_registry import create_node, known_nodes


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
    parser = argparse.ArgumentParser(description="OMX Control 백엔드")
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
    requested_nodes: list[str] = list(cfg.get("nodes", []))
    bridge_cfg: dict[str, Any] = cfg.get("bridge", {}) or {}

    logger.info("=== OMX Control 시작 (host=%s) ===", host_name)
    logger.info("실행할 노드: %s", requested_nodes)

    unknown = [n for n in requested_nodes if n not in known_nodes()]
    if unknown:
        raise ValueError(f"알 수 없는 노드: {unknown}. 등록된 노드: {known_nodes()}")

    # ─── Zenoh 세션 초기화 ────────────────────────────────────
    ZenohSession.init(cfg.get("zenoh"))

    # ─── joint_offsets 로드 (BA 결과 파일이 있으면) ────────────
    # raw_to_rad/rad_to_raw 호출 전에 가시성 위해 eager load.
    from core import joint_offsets as _jo

    _jo.reload()

    # ─── D405 intrinsic seed (camera 노드 있을 때만) ──────────
    if "camera" in requested_nodes:
        from modules.calibration.loader import CALIB_DIR
        from modules.camera.factory_intrinsic import seed_d405_intrinsic_if_missing

        seed_d405_intrinsic_if_missing(CALIB_DIR / "intrinsic.npz")

    # ─── 노드 인스턴스 생성 ────────────────────────────────────
    instances: dict[str, Any] = {}
    for name in requested_nodes:
        instances[name] = create_node(name)

    # ─── 노드 시작 ────────────────────────────────────────────
    for name, node in instances.items():
        node.start()
        logger.info("노드 시작됨: %s (%s)", name, node.node_name)

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
