"""host_mock 단일 process e2e — Scene3D snapshot + Storage put + ScanTask 진행.

scene3d_decoupling.md §13.1 e2e (host_mock).

전제: backend `uv run python main.py --host mock` 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리.

검증:
1. SCENE3D_SET_STREAM(enabled=true) — refcount + mock depth publish 시작
2. SCENE3D_SNAPSHOT — consensus N=3 frame
3. STORAGE_NEW_SCAN_SESSION + PUT_SCAN — RDB row + ObjectStore blob round-trip
4. STORAGE_LIST_SCANS — 방금 PUT 한 row 확인
5. SCENE3D_SET_STREAM(enabled=false) — refcount 0 → CAMERA disable
6. (optional) TASK_RUN(task=scan) — full ScanTask 자체 자리 (BuildReconstruction
   자체 자리 hand_eye 없어서 fail 자체 자리 자체 자리 자체 자리 자체 자리 — 본 자체 자리
   step tree / step_result publish 만 확인).
"""

from __future__ import annotations

import base64
import json
import sys
import time

import zenoh


ROBOT_ID = "so101_6dof_0"


def _call(session, key: str, data: dict, timeout: float = 10.0):
    payload = json.dumps({"timestamp": time.time(), "data": data}).encode()
    replies = session.get(key, payload=payload, timeout=timeout)
    for reply in replies:
        if reply.ok is not None:
            return json.loads(reply.ok.payload.to_bytes())
        if reply.err is not None:
            return {
                "success": False,
                "message": f"err: {reply.err.payload.to_bytes()}",
            }
    return {"success": False, "message": "no reply"}


def main() -> int:
    session = zenoh.open(zenoh.Config())
    time.sleep(2)

    failures: list[str] = []

    # 1. set_stream ON — refcount + mock depth publish
    print("->SCENE3D_SET_STREAM(enabled=true)")
    res = _call(
        session, f"horibot/{ROBOT_ID}/scene3d/srv/set_stream", {"enabled": True}
    )
    print(f"  <-success={res.get('success')}, message={res.get('message')}")
    if not res.get("success"):
        failures.append("SET_STREAM enable")

    # 2. depth_frame 자체 자리 자체 자리 settle wait (mock 8 FPS, N=3 자체 자리 자체 자리)
    time.sleep(0.6)

    # 3. SCENE3D_SNAPSHOT
    print("->SCENE3D_SNAPSHOT (num_frames=3)")
    res = _call(
        session,
        f"horibot/{ROBOT_ID}/scene3d/srv/snapshot",
        {"num_frames": 3, "timeout_s": 5.0},
        timeout=15.0,
    )
    snap_ok = res.get("success", False)
    print(f"  <-success={snap_ok}, message={res.get('message')}")
    if snap_ok:
        snap = res["data"]
        jpeg_len = len(base64.b64decode(snap["color_bgr_jpeg"]))
        zstd_len = len(base64.b64decode(snap["depth_z16_zstd"]))
        print(
            f"     intrinsic={snap['intrinsic']['width']}x{snap['intrinsic']['height']}"
            f" fx={snap['intrinsic']['fx']}"
            f"  color_jpeg={jpeg_len} bytes  depth_zstd={zstd_len} bytes"
            f"  motors={snap['motor_positions']}"
        )
    else:
        failures.append("SNAPSHOT")
        snap = None

    # 4. STORAGE_NEW_SCAN_SESSION
    print("->STORAGE_NEW_SCAN_SESSION")
    res = _call(
        session,
        "horibot/storage/srv/scan/new_session",
        {"robot_id": ROBOT_ID, "session_id": "", "label": "mock_e2e"},
    )
    print(f"  <-success={res.get('success')}, message={res.get('message')}")
    if not res.get("success"):
        failures.append("NEW_SCAN_SESSION")
        session_row_id = None
    else:
        session_row_id = res["data"]["session"]["id"]
        print(f"     session_row_id={session_row_id}")

    # 5. STORAGE_PUT_SCAN — blob_bytes 자체 자리 자체 자리 (color jpeg + depth zstd concat)
    if snap is not None and session_row_id is not None:
        # scan_blob.encode format: [u32 jpeg_len LE][jpeg][zstd]
        import struct
        color_jpeg_bytes = base64.b64decode(snap["color_bgr_jpeg"])
        depth_zstd_bytes = base64.b64decode(snap["depth_z16_zstd"])
        blob = (
            struct.pack("<I", len(color_jpeg_bytes))
            + color_jpeg_bytes
            + depth_zstd_bytes
        )
        print(f"→ STORAGE_PUT_SCAN (session={session_row_id}, blob={len(blob)} bytes)")
        res = _call(
            session,
            "horibot/storage/srv/scan/put",
            {
                "session_row_id": session_row_id,
                "blob_bytes": base64.b64encode(blob).decode(),
                "num_frames": snap["num_frames"],
                "width": snap["intrinsic"]["width"],
                "height": snap["intrinsic"]["height"],
                "fx": snap["intrinsic"]["fx"],
                "fy": snap["intrinsic"]["fy"],
                "cx": snap["intrinsic"]["cx"],
                "cy": snap["intrinsic"]["cy"],
                "depth_scale": snap["intrinsic"]["depth_scale"],
                "motor_positions": snap["motor_positions"],
                "arm_motor_ids": snap["arm_motor_ids"],
            },
            timeout=15.0,
        )
        print(f"  <-success={res.get('success')}, message={res.get('message')}")
        if not res.get("success"):
            failures.append("PUT_SCAN")
        else:
            scan = res["data"]["scan"]
            print(
                f"     scan_row_id={scan['id']}, scan_id={scan['scan_id']},"
                f" blob_key={scan['blob_key']}"
            )

        # 6. LIST_SCANS
        print("->STORAGE_LIST_SCANS")
        res = _call(
            session,
            "horibot/storage/srv/scan/list",
            {"session_row_id": session_row_id},
        )
        print(f"  <-success={res.get('success')}")
        if res.get("success"):
            scans = res["data"]["scans"]
            print(f"     count={len(scans)}")
            if len(scans) != 1:
                failures.append(
                    f"LIST_SCANS count mismatch — {len(scans)} != 1"
                )
        else:
            failures.append("LIST_SCANS")

    # 7. SET_STREAM OFF
    print("->SCENE3D_SET_STREAM(enabled=false)")
    res = _call(
        session, f"horibot/{ROBOT_ID}/scene3d/srv/set_stream", {"enabled": False}
    )
    print(f"  <-success={res.get('success')}, message={res.get('message')}")
    if not res.get("success"):
        failures.append("SET_STREAM disable")

    session.close()

    print()
    if failures:
        print(f"[FAIL] ({len(failures)}):")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("[PASS] Scene3D snapshot + Storage put round-trip OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
