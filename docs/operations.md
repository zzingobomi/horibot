# Operations

PC + Pi 두 대로 분산 운영. 단일 머신 모드(`host_dev`)는 모든 노드를 PC 한 대에 띄움 — 개발/회귀용.

---

## 머신 분배

| 머신            | IP            | OS            | 노드                                                       |
| --------------- | ------------- | ------------- | ---------------------------------------------------------- |
| PC (개발기)     | (LAN 동적)    | Windows 11    | calibration, task, detector, pointcloud, bridge, (gamepad) |
| 모터 Pi         | 192.168.0.101 | Ubuntu 22.04  | motor, motion                                              |
| 카메라 Pi       | 192.168.0.102 | Ubuntu 22.04  | camera                                                     |

분배의 핵심 이유:

- (a) USB 대역폭 경합 해소 — D405와 OpenRB-150이 한 USB 컨트롤러를 공유하지 않음.
- (b) 100Hz 제어 명령(`MOTOR_CMD_JOINT`)을 네트워크로 안 보냄 — TrajectoryRunner와 MotorNode 같은 머신.
- (c) 무거운 연산(YOLO, Open3D, PyBullet PC 측, TSDF build)은 PC.

---

## Pi 설정

- Ubuntu 22.04 aarch64, uv 설치 완료 (둘 다).
- 일반 의존성은 `uv sync --only-group <role>` 한 줄로 끝남 (`<role>` = `pi-motor` 또는 `pi-camera`).
- **두 라이브러리만 특수 처리**:
  - `pyrealsense2` — aarch64 PyPI wheel 없음 → 카메라 Pi에서 소스 빌드 후 별도 install. 빌드 절차는 [docs/pyrealsense2-build-guide.md](pyrealsense2-build-guide.md). 빌드한 wheel은 `uv pip install ./pyrealsense2-*.whl`로 별도 설치 + 그룹 sync 시 `--no-install-package pyrealsense2` 명시.
  - `open3d` — aarch64 wheel 이슈 있지만 PC에서만 필요하니 Pi에서는 무관.

```powershell
# 카메라 Pi 시퀀스 (pyrealsense2 빌드/설치 끝난 상태 가정)
uv sync --only-group pi-camera --no-install-package pyrealsense2
uv run --no-sync python main.py --host pi_camera

# 모터 Pi
uv sync --only-group pi-motor
uv run --no-sync python main.py --host pi_motor
```

---

## 호스트 자동 감지

[backend/main.py:35](../backend/main.py#L35) — hostname을 lowercase + `-`→`_` 정규화 후 `host_<hostname>.yaml` 매칭. 매칭 실패 시 `host_dev.yaml` fallback. Pi의 hostname을 `pi_motor` / `pi_camera`로 맞춰두면 `--host` 인자 없이도 정확한 config가 잡힘.

---

## 네트워크

- Zenoh peer 모드. 같은 LAN + 멀티캐스트 정상이면 노드들끼리 자동 발견.
- 라우터/스위치가 멀티캐스트를 막거나 IPv6/멀티 NIC 환경이면 host config의 `zenoh.connect`에 endpoint 명시 (`tcp/<ip>:7447`).
- 방화벽 룰 고정용 `zenoh.listen` 포트도 host config에 정의 가능.
