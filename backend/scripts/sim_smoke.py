"""분산 sim smoke — 3 process (pc / pi_motor / pi_camera) 간 Zenoh service call.

scene3d_decoupling.md §13.1 분산 sim 자리. memory anchor "mock 통과 ≠
distributed 검증" — cross-process state 가 single-process e2e 로 안 잡힘.

본 script 는 4번째 process (별도 Zenoh peer) — 3 sim process 와 같은 peer
group 자리 자체 자리.

검증:
- STORAGE_NEW_SCAN_SESSION + LIST_SCAN_SESSIONS — pc_sim 의 storage_node
- CAMERA_SET_DEPTH_STREAM — pi_camera_sim 의 mock_camera (cross-process)
- MOTION_GET_TCP — pi_motor_sim 의 motion_node (cross-process)
"""

from __future__ import annotations

import json
import sys
import time

import zenoh


def _call(session, key: str, data: dict, timeout: float = 5.0):
    payload = json.dumps({"timestamp": time.time(), "data": data}).encode()
    replies = session.get(key, payload=payload, timeout=timeout)
    for reply in replies:
        if reply.ok is not None:
            return json.loads(reply.ok.payload.to_bytes())
        if reply.err is not None:
            return {"success": False, "message": f"err: {reply.err.payload.to_bytes()}"}
    return {"success": False, "message": "no reply (peer X)"}


def main() -> int:
    session = zenoh.open(zenoh.Config())
    # peer 발견 wait
    time.sleep(3)

    failures: list[str] = []

    # 1. STORAGE_NEW_SCAN_SESSION (pc_sim)
    print("->STORAGE_NEW_SCAN_SESSION (pc_sim)")
    res = _call(
        session,
        "horibot/storage/srv/scan/new_session",
        {"robot_id": "so101_6dof_0", "session_id": "sim_test", "label": "sim"},
    )
    print(f"  <-success={res.get('success')}, message={res.get('message')}")
    if not res.get("success"):
        failures.append("STORAGE_NEW_SCAN_SESSION")
        session_row_id = None
    else:
        session_row_id = res["data"]["session"]["id"]
        print(f"     session_row_id={session_row_id}")

    # 2. STORAGE_LIST_SCAN_SESSIONS — round-trip
    print("->STORAGE_LIST_SCAN_SESSIONS (pc_sim)")
    res = _call(
        session,
        "horibot/storage/srv/scan/list_sessions",
        {"robot_id": "so101_6dof_0", "limit": 10},
    )
    print(f"  <-success={res.get('success')}")
    if res.get("success"):
        sessions = res["data"]["sessions"]
        print(f"     sessions count={len(sessions)}")
        if session_row_id is not None and not any(
            s["id"] == session_row_id for s in sessions
        ):
            failures.append(
                "STORAGE_LIST_SCAN_SESSIONS — 방금 INSERT 한 session row 안 보임"
            )
    else:
        failures.append("STORAGE_LIST_SCAN_SESSIONS")

    # 3. CAMERA_SET_DEPTH_STREAM (pi_camera_sim, cross-process)
    print("->CAMERA_SET_DEPTH_STREAM(so101_6dof_0, enabled=true) (pi_camera_sim)")
    res = _call(
        session,
        "horibot/so101_6dof_0/camera/srv/set_depth_stream",
        {"enabled": True},
    )
    print(f"  <-success={res.get('success')}, message={res.get('message')}")
    if not res.get("success"):
        failures.append("CAMERA_SET_DEPTH_STREAM (cross-process)")

    # 4. MOTION_GET_TCP (pi_motor_sim, cross-process)
    print("->MOTION_GET_TCP(so101_6dof_0) (pi_motor_sim)")
    res = _call(
        session,
        "horibot/so101_6dof_0/motion/srv/get_tcp",
        {},
        timeout=10.0,
    )
    print(f"  <-success={res.get('success')}, message={res.get('message')}")
    if not res.get("success"):
        failures.append("MOTION_GET_TCP (cross-process)")

    session.close()

    print()
    if failures:
        print(f"[FAIL] ({len(failures)}):")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("[PASS] Zenoh peer + cross-process service call OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
