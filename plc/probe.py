"""OpenPLC 런타임 Modbus :502 진단/핸드셰이크 테스트 (버려도 됨).

실행:
  uv run --with pymodbus --no-project python plc/probe.py          # 현재 코일 상태 read
  uv run --with pymodbus --no-project python plc/probe.py --pick   # 마스터 집기응답 1사이클(pick_done 펄스)

coil 0/1/2 = conveyor_run / object_arrived / pick_done (§9.4 매핑, 실측 확정).
sensor(%IX0.0)는 Discrete Input = 마스터가 못 씀 → Editor 디버그 force로 주입.
※ write 후 상태확인은 PLC 스캔(20ms) 뒤에 — 즉시 read 하면 반영 전 값을 잡음.
"""

import argparse
import time

from pymodbus.client import ModbusTcpClient

HOST, PORT = "127.0.0.1", 502
NAMES = {0: "conveyor_run", 1: "object_arrived", 2: "pick_done"}
PICK_DONE = 2
SCAN_WAIT = 0.1  # PLC 스캔(20ms)의 5배 여유


def snapshot(client: ModbusTcpClient, label: str) -> None:
    rr = client.read_coils(0, count=3)
    if rr.isError():
        print(f"[{label}] read 에러: {rr}")
        return
    print(f"[{label}] " + ", ".join(f"{NAMES[a]}={rr.bits[a]}" for a in NAMES))


def do_pick(client: ModbusTcpClient) -> None:
    """마스터 집기응답 1사이클: pick_done 펄스(True → 스캔대기 → False)."""
    snapshot(client, "before")
    client.write_coil(PICK_DONE, True)
    print("[write ] pick_done <- True  (집기완료 신호)")
    time.sleep(SCAN_WAIT)
    snapshot(client, "after ")  # object_arrived 리셋 + conveyor 재가동 확인
    client.write_coil(PICK_DONE, False)
    print("[write ] pick_done <- False (신호 해제)")
    time.sleep(SCAN_WAIT)
    snapshot(client, "reset ")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--pick",
        action="store_true",
        help="마스터 집기응답 1사이클(pick_done 펄스)",
    )
    args = ap.parse_args()

    client = ModbusTcpClient(HOST, port=PORT)
    if not client.connect():
        print(f"connect fail: {HOST}:{PORT} (런타임 RUNNING인지 확인)")
        return
    try:
        do_pick(client) if args.pick else snapshot(client, "read  ")
    finally:
        client.close()


if __name__ == "__main__":
    main()
