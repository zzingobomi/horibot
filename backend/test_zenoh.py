import time
import json
import math
import cv2
import numpy as np
import zenoh

from core.topic_map import Topic, Service
from core.units import raw_to_deg

PUBLISH_HZ = 20

# ─────────────────────────────────────────────
# Motor
# ─────────────────────────────────────────────

MOTOR_IDS = [
    (1, "joint1"),
    (2, "joint2"),
    (3, "joint3"),
    (4, "joint4"),
    (5, "joint5"),
    (6, "gripper"),
]

cmd_positions = {mid: 2048 for mid, _ in MOTOR_IDS}
actual_positions = {mid: 2048.0 for mid, _ in MOTOR_IDS}
FOLLOW_SPEED = 0.05


# ─────────────────────────────────────────────
# Motion (TCPPose: tuple 기반)
# ─────────────────────────────────────────────

current_tcp = {
    "position": (0.3, 0.0, 0.2),  # Position3
    "quaternion": (0.0, 0.0, 0.0, 1.0),  # Quaternion
}

target_tcp = {
    "position": (0.3, 0.0, 0.2),
    "quaternion": (0.0, 0.0, 0.0, 1.0),
}

TCP_FOLLOW_SPEED = 0.05


def lerp(a, b, t):
    return a + (b - a) * t


def lerp_tuple(a, b, t):
    return tuple(lerp(ai, bi, t) for ai, bi in zip(a, b))


def update_tcp():
    global current_tcp
    current_tcp["position"] = lerp_tuple(
        current_tcp["position"],
        target_tcp["position"],
        TCP_FOLLOW_SPEED,
    )
    # quaternion은 mock이라 그대로 둠


# ─────────────────────────────────────────────
# Fake Camera
# ─────────────────────────────────────────────


def make_fake_frame(t: float) -> bytes:
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:] = (
        int(40 + 20 * math.sin(t)),
        int(40 + 20 * math.sin(t + 2)),
        int(40 + 20 * math.sin(t + 4)),
    )

    cv2.putText(
        frame,
        "MOCK CAMERA",
        (20, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"t={t:.2f}",
        (20, 100),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (180, 180, 180),
        1,
    )

    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return buf.tobytes()


# ─────────────────────────────────────────────
# Motor Handlers
# ─────────────────────────────────────────────


def on_cmd_joint(sample):
    data = json.loads(sample.payload.to_bytes())

    for j in data.get("joints", []):
        if "id" in j and "position" in j:
            cmd_positions[j["id"]] = j["position"]
            print(f"[mock] CMD joint{j['id']} → {j['position']}")


def on_enable(query):
    data = json.loads(query.payload.to_bytes()) if query.payload else {}
    enable = data.get("data", {}).get("enable", True)

    print(f"[mock] MOTOR_ENABLE: {enable}")

    query.reply(
        query.key_expr, json.dumps({"success": True, "data": {"enable": enable}})
    )


def on_get_config(query):
    configs = [
        {
            "id": mid,
            "name": name,
            "model": "XL430-W250",
            "mode": "position",
            "home": 2048,
            "limit": {"min": 0, "max": 4095},
        }
        for mid, name in MOTOR_IDS
    ]

    query.reply(
        query.key_expr, json.dumps({"success": True, "data": {"motors": configs}})
    )


# ─────────────────────────────────────────────
# Motion Handlers (🔥 tuple 기반)
# ─────────────────────────────────────────────


def on_get_tcp(query):
    query.reply(
        query.key_expr,
        json.dumps(
            {
                "success": True,
                "data": {
                    "position": list(current_tcp["position"]),
                    "quaternion": list(current_tcp["quaternion"]),
                },
            }
        ),
    )


def on_move_tcp(query):
    global target_tcp

    data = json.loads(query.payload.to_bytes())
    tcp = data.get("data", {})

    if "position" in tcp:
        p = tcp["position"]
        target_tcp["position"] = (
            float(p[0]),
            float(p[1]),
            float(p[2]),
        )

    if "quaternion" in tcp:
        q = tcp["quaternion"]
        target_tcp["quaternion"] = (
            float(q[0]),
            float(q[1]),
            float(q[2]),
            float(q[3]),
        )

    print(f"[mock] MOVE_TCP → {target_tcp}")

    query.reply(query.key_expr, json.dumps({"success": True, "message": "moving"}))


def on_move_l(query):
    print("[mock] MOVE_L")
    return on_move_tcp(query)


def on_stop(query):
    global target_tcp

    target_tcp = {
        "position": current_tcp["position"],
        "quaternion": current_tcp["quaternion"],
    }

    print("[mock] STOP")

    query.reply(query.key_expr, json.dumps({"success": True}))


# ─────────────────────────────────────────────
# Calibration Handlers
# ─────────────────────────────────────────────


def on_calib_capture(query):
    print("[mock] CAPTURE")

    query.reply(
        query.key_expr,
        json.dumps({"success": True, "data": {"index": int(time.time())}}),
    )


def on_handeye_capture(query):
    print("[mock] HANDEYE CAPTURE")

    query.reply(query.key_expr, json.dumps({"success": True}))


def on_handeye_commit(query):
    print("[mock] HANDEYE COMMIT")

    query.reply(
        query.key_expr,
        json.dumps(
            {
                "success": True,
                "data": {
                    "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                    "t": [0.1, 0.0, 0.2],
                },
            }
        ),
    )


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────


def main():
    session = zenoh.open(zenoh.Config())

    pub_motor = session.declare_publisher(Topic.MOTOR_STATE_JOINT)
    pub_motion = session.declare_publisher(Topic.MOTION_STATE_TRAJ)

    session.declare_subscriber(Topic.MOTOR_CMD_JOINT, on_cmd_joint)

    session.declare_queryable(Service.MOTOR_ENABLE, on_enable)
    session.declare_queryable(Service.MOTOR_GET_CONFIG, on_get_config)

    session.declare_queryable(Service.MOTION_GET_TCP, on_get_tcp)
    session.declare_queryable(Service.MOTION_MOVE_TCP, on_move_tcp)
    session.declare_queryable(Service.MOTION_MOVE_L, on_move_l)
    session.declare_queryable(Service.MOTION_STOP, on_stop)

    session.declare_queryable(Service.CALIB_CAPTURE, on_calib_capture)
    session.declare_queryable(Service.CALIB_HANDEYE_CAPTURE, on_handeye_capture)
    session.declare_queryable(Service.CALIB_HANDEYE_COMMIT, on_handeye_commit)

    print("[mock] started")

    interval = 1.0 / PUBLISH_HZ

    try:
        while True:
            t = time.time()

            # ─── Motor State ─────────────────────
            for mid, _ in MOTOR_IDS:
                actual_positions[mid] += (
                    cmd_positions[mid] - actual_positions[mid]
                ) * FOLLOW_SPEED

            joints = [
                {
                    "id": mid,
                    "name": name,
                    "position": round(actual_positions[mid]),
                    "degree": raw_to_deg(round(actual_positions[mid])),
                }
                for mid, name in MOTOR_IDS
            ]

            pub_motor.put(json.dumps({"timestamp": t, "joints": joints}))

            # ─── Motion State ────────────────────
            update_tcp()

            pub_motion.put(
                json.dumps(
                    {
                        "timestamp": t,
                        "tcp": {
                            "position": list(current_tcp["position"]),
                            "quaternion": list(current_tcp["quaternion"]),
                        },
                    }
                )
            )

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n[mock] stopped")
    finally:
        session.close()


if __name__ == "__main__":
    main()
