"""터치 테스트 — 사선/부양의 원인이 FK(로봇 기구학)인지 hand_eye(카메라)인지 판별.

원리: 재구성(카메라)은 바닥을 3~7° 기울고 +15~20mm 뜬 것으로 본다. 그런데 이 오차는
      FK_오차 ⊕ hand_eye_오차 의 합이다. 카메라를 빼고 **로봇이 자기 FK 로 바닥을
      어디로 보는지** 직접 재면 둘을 분리할 수 있다:
  - 로봇 TCP 로 바닥 여러 점을 실제로 찍고, 그 순간 FK 가 보고하는 좌표를 기록.
  - 그 점들이 이루는 평면이 **평평(수평)** 하면 → 로봇 FK 는 정확 → 사선은 100% hand_eye.
  - 그 평면이 **기울어져** 있으면 → 로봇 기구학(FK/base)이 사선에 기여.

★ 안전: 이 스크립트는 **읽기 전용**. TCP_STATE 스트림만 구독하며 로봇에 어떤 명령도
  보내지 않는다 (motion publish 0). 팔은 사용자가 직접(토크 off 백드라이브 or UI jog)
  움직인다.

실행:
  1) backend(분산: PC + pi) 가 떠 있어야 함. bridge = PC :8000.
  2) SO-101 팔을 손으로 움직일 수 있게 (calibrate/move 페이지에서 토크 off, 또는 jog).
  3) .venv\Scripts\python.exe scripts\grasp_verify\touch_test.py
  4) 그리퍼(닫은 상태)를 **수직 아래로 향한 자세 유지**한 채 테이블 바닥의 서로 떨어진
     점 3~5곳을 살짝 찍고, 찍을 때마다 Enter. 자세를 최대한 일정하게(수직 다운) 유지할 것
     — 그래야 tool 오프셋이 상수가 되어 '기울기'가 순수 FK 오차가 됨.
  5) 다 찍으면 q + Enter → 평면 피팅 결과.

인자: [host:port] [robot_id]  (기본 localhost:8000 so101_6dof_0)
"""
from __future__ import annotations

import asyncio
import json
import os
import struct
import sys

import numpy as np
import msgspec
import websockets

HOST = sys.argv[1] if len(sys.argv) > 1 else "localhost:8000"
ROBOT = sys.argv[2] if len(sys.argv) > 2 else "so101_6dof_0"
URL = f"ws://{HOST}/ws"
TOPIC = f"stream/motion/{ROBOT}/tcp_state"

_latest: dict | None = None


def decode_frame(data: bytes):
    ver, ftype, klen = struct.unpack_from(">BBH", data, 0)
    key = data[4 : 4 + klen].decode("utf-8")
    return ftype, key, data[4 + klen :]


async def _reader(ws):
    global _latest
    async for msg in ws:
        if isinstance(msg, (bytes, bytearray)):
            ftype, _key, payload = decode_frame(bytes(msg))
            if ftype == 1:
                try:
                    _latest = msgspec.msgpack.decode(payload)
                except Exception:
                    pass


def _fit_plane(pts: np.ndarray):
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    n = vt[2]
    if n[2] < 0:
        n = -n
    tilt = np.degrees(np.arccos(np.clip(abs(n[2]), 0, 1)))
    return n, c, tilt


async def main():
    print(f"[연결] {URL}  topic={TOPIC}")
    async with websockets.connect(URL, max_size=None) as ws:
        await ws.send(json.dumps({"op": "subscribe", "topic": TOPIC}))
        asyncio.create_task(_reader(ws))

        # 첫 샘플 대기
        for _ in range(50):
            if _latest is not None:
                break
            await asyncio.sleep(0.1)
        if _latest is None:
            print("✗ TCP_STATE 수신 실패 — backend/motion(pi_hori1) 떠 있는지, robot_id 확인")
            return

        ca = _latest.get("calibration_applied")
        cs = _latest.get("calibration_stale")
        print(f"[TCP_STATE 수신 OK] calibration_applied={ca} calibration_stale={cs}")
        if ca is False:
            print("  ⚠ calibration_applied=False — 이 경우 그 자체로 사선/부양 (FK 무보정)")
        print("\n그리퍼 닫고 '수직 아래' 자세로 바닥 점을 찍은 뒤 Enter. 3~5점. 끝나면 q+Enter.\n")

        os.makedirs("debug", exist_ok=True)
        save_path = "debug/touch_test_captures.json"

        def _save(caps: list[dict]) -> str:
            with open(save_path, "w", encoding="utf-8") as f:
                json.dump({"robot_id": ROBOT, "captures": caps}, f, indent=1)
            return save_path

        loop = asyncio.get_event_loop()
        caps: list[dict] = []
        while True:
            cur = _latest
            zmm = cur["position"][2] * 1000 if cur else float("nan")
            cmd = await loop.run_in_executor(
                None,
                input,
                f"\n[현재 TCP z={zmm:+.1f}mm | 지금까지 {len(caps)}점] Enter=이 지점 찍기 / q=종료 > ",
            )
            if cmd.strip().lower() == "q":
                break
            cur = _latest
            if cur is None:
                print("  ✗ 아직 TCP 데이터 없음 — 다시")
                continue
            caps.append(cur)
            p = cur["position"]
            q = cur["quaternion"]
            try:
                _save(caps)
                saved = "저장됨"
            except Exception as e:
                saved = f"저장실패({e})"
            print(
                f"  ✓✓✓ {len(caps)}번째 점 찍힘! "
                f"pos=({p[0]*1000:.1f}, {p[1]*1000:.1f}, z={p[2]*1000:.1f})mm  [{saved}]"
            )

        print(f"\n총 {len(caps)}점 → debug/touch_test_captures.json")
        if len(caps) < 3:
            print(f"점 {len(caps)}개 — 평면 피팅엔 최소 3점 필요. 종료.")
            return

        pts = np.array([c["position"] for c in caps])
        quats = np.array([c["quaternion"] for c in caps])
        n, c, tilt = _fit_plane(pts)
        zmm = pts[:, 2] * 1000

        # 자세 일관성 체크 (tool 오프셋 상수 가정 검증)
        qdev = np.degrees(
            2 * np.arccos(np.clip(np.abs(quats @ quats[0]), 0, 1))
        ).max()

        print("\n================ 결과 ================")
        print(f"찍은 점 {len(caps)}개, FK-바닥 평면:")
        print(f"  법선 = ({n[0]:+.3f}, {n[1]:+.3f}, {n[2]:+.3f})")
        print(f"  수평 대비 기울기 = {tilt:.2f}°   (0°=완전 수평)")
        print(f"  점별 z(mm) = {np.array2string(zmm, precision=1)}  (평면두께 {zmm.max()-zmm.min():.1f}mm)")
        print(f"  자세 최대 편차 = {qdev:.1f}° (작을수록 tool 오프셋 상수 → 기울기 해석 신뢰)")
        print("\n판정:")
        print("  • 기울기 ≲ 1.5° → 로봇 FK 는 바닥을 평평하게 봄 = FK 정확.")
        print("    → 재구성의 3~7° 사선은 hand_eye(카메라 외부캘) 탓. fix = hand_eye 재정정.")
        print("  • 기울기 ≈ 3~7° (재구성과 비슷) → 로봇 기구학(FK/base)이 사선에 기여.")
        print("    → hand_eye 만으론 해결 안 됨. 기구학/base frame 점검.")
        if qdev > 15:
            print(f"  ⚠ 자세 편차 {qdev:.0f}° 큼 — 기울기 값에 tool 오프셋 회전이 섞였을 수 있음."
                  " 자세를 더 일정히(수직 다운) 유지해 재측정 권장.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
