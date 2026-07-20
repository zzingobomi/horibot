"""teach_capture — 토크오프 손 시연 캡처 (Enter = 그 순간 관절+TCP 저장).

    uv run --no-sync python scripts/teach_capture.py \
        [--robot so101_6dof_0] [--deploy pc] [--out debug/teach/<ts>.jsonl]

용도 (docs/motion.md §11): 실물에서 토크오프로 팔을 손으로 움직여 "이 위치의
물체를 나는 이렇게 잡는다" 자세를 만든 뒤 Enter — 그 순간의 motion tcp_state
(URDF rad 관절 + TCP pose, 캘 적용 FK)를 jsonl 로 적재. 수집된 시연 자세가
후보 생성(grasp_families 격자)/도달성 판정의 **물리 ground truth** — "새
시스템이 사용자가 손으로 잡은 자세를 전부 찾는가"가 수술 합격 판정.

동작: runtime 부팅 없음 — Zenoh transport 만 조인해 tcp_state 구독 (구독
전용, 모션 명령 0 — 실 robot 안전). backend(pi/pc)가 떠 있어야 스트림이 온다.
토크오프여도 인코더 위치는 정상 publish (STS3215 position read 는 토크 무관).

입력: Enter = 저장 / 텍스트+Enter = 그 텍스트를 label 로 저장 (예: "cube
x0.19 y-0.10 우하단") / q+Enter = 종료.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import UTC, datetime
from pathlib import Path

BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))

from apps.main import load_configs  # noqa: E402
from framework.contract.publisher import decode_event  # noqa: E402
from modules.motion.contract import TcpState  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="토크오프 시연 캡처")
    parser.add_argument("--robot", default="so101_6dof_0")
    parser.add_argument(
        "--deploy", default="pc",
        help="zenoh 설정을 빌릴 deployment (pc = LAN mesh 조인)",
    )
    parser.add_argument("--out", default="")
    args = parser.parse_args()
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(  # type: ignore[attr-defined]
                encoding="utf-8", errors="replace"
            )

    out = Path(args.out) if args.out else (
        BACKEND_ROOT / "debug" / "teach"
        / f"{datetime.now(UTC).strftime('%Y%m%d_%H%M%S')}.jsonl"
    )
    out.parent.mkdir(parents=True, exist_ok=True)

    deploy, _robots = load_configs(args.deploy)
    from infra.transport.zenoh import ZenohTransport

    transport = ZenohTransport(deploy.zenoh)
    latest: list[TcpState] = []

    def on_tcp(payload: bytes) -> None:
        latest[:] = [decode_event(TcpState, payload)]

    key = f"stream/motion/{args.robot}/tcp_state"
    handle = transport.subscribe(key, on_tcp)
    print(f"구독: {key} → 저장: {out}")
    print("Enter=저장 / 텍스트+Enter=label 과 함께 저장 / q+Enter=종료")

    stop = threading.Event()
    n = 0
    try:
        # 첫 수신 대기 (backend 미기동/robot_id 오타를 침묵 대신 즉시 표면화)
        t0 = time.time()
        while not latest:
            if time.time() - t0 > 10.0:
                print("⚠ 10s 동안 tcp_state 없음 — backend 떠 있는지 / "
                      "robot_id 맞는지 확인 (계속 대기)")
                t0 = time.time()
            time.sleep(0.2)
        print(f"스트림 수신 중 (seq={latest[0].seq}) — 캡처 시작 가능")
        while not stop.is_set():
            line = input("> ").strip()
            if line.lower() == "q":
                break
            if not latest:
                print("  (수신 없음 — skip)")
                continue
            st = latest[0]
            age = time.time() - st.timestamp_unix
            rec = {
                "ts": datetime.now(UTC).isoformat(),
                "label": line,
                "robot_id": st.robot_id,
                "joint_names": list(st.joint_names),
                "joints": [round(v, 6) for v in st.joints],
                "position": [round(v, 6) for v in st.position],
                "quaternion": [round(v, 6) for v in st.quaternion],
                "state_age_s": round(age, 3),
            }
            with open(out, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            n += 1
            p = st.position
            print(
                f"  #{n} 저장 — tcp=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
                f"joints={[round(v, 2) for v in st.joints]}"
                + (f" (⚠ 스트림 {age:.1f}s 지연)" if age > 1.0 else "")
            )
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        try:
            handle.undeclare()
        except Exception:  # noqa: BLE001 — 종료 정리 best-effort
            pass
        transport.close()
    print(f"\n종료 — {n}개 저장: {out}")


if __name__ == "__main__":
    main()
