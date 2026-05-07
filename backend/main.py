import logging
import signal
import sys
import uvicorn


from core.zenoh_session import ZenohSession
from nodes.motor_node import MotorNode
from nodes.camera_node import CameraNode
from nodes.motion_node import MotionNode
from nodes.calibration_node import CalibrationNode
from nodes.task_node import TaskNode
from nodes.detector_node import DetectorNode
from nodes.gamepad_node import GamepadNode
from bridge.zenoh_bridge import app, setup_zenoh_subscribers

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s - %(message)s",
)
logger = logging.getLogger(__name__)

BRIDGE_HOST = "0.0.0.0"
BRIDGE_PORT = 8000


def main():
    logger.info("=== OMX Control 시작 ===")

    # ─── Zenoh 세션 초기화 ────────────────────────────────────
    ZenohSession.init()

    # ─── 노드 초기화 ─────────────────────────────────────────
    motor_node = MotorNode()
    camera_node = CameraNode()
    motion_node = MotionNode()
    calib_node = CalibrationNode(camera=camera_node.camera)
    task_node = TaskNode(camera=camera_node.camera)
    detector_node = DetectorNode(camera=camera_node.camera)
    # FIXME: gamepad_node 활성화하면 기존 모터 제어 방식과 충돌하는듯 (개선 필요)
    # gamepad_node = GamepadNode()

    # ─── 노드 시작 (별도 스레드) ──────────────────────────────
    nodes = [motor_node, camera_node, motion_node,
             calib_node, task_node, detector_node]
    for node in nodes:
        node.start()
        logger.info(f"노드 시작됨: {node.node_name}")

    # ─── Zenoh → WebSocket 구독 설정 ─────────────────────────
    setup_zenoh_subscribers()

    # ─── 종료 시그널 처리 ─────────────────────────────────────
    def shutdown(sig, frame):
        logger.info("종료 신호 수신, 노드 정리 중...")
        for node in nodes:
            node.stop()
        ZenohSession.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ─── FastAPI 브릿지 서버 시작 ──────────────────────────────
    logger.info(f"브릿지 서버 시작: ws://{BRIDGE_HOST}:{BRIDGE_PORT}")
    uvicorn.run(app, host=BRIDGE_HOST, port=BRIDGE_PORT, log_level="warning")


if __name__ == "__main__":
    main()
