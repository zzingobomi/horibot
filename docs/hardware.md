# Hardware & Operations

로봇 2대의 물리 HW + 개조 기록 + 머신 토폴로지/운영 통합 문서 (2026-07-11 — 옛 so101_6dof_plan.md / distributed_topology.md / operations.md 를 본 문서로 통합, 원문 상세는 git history).

모터 실 스펙(모델/home/limit/profile) SSOT = [robot/omx_f/motors.yaml](../robot/omx_f/motors.yaml) / [robot/so101_6dof/motors.yaml](../robot/so101_6dof/motors.yaml).

---

## 1. 머신 토폴로지 + 운영

PC 1대 + Raspberry Pi 3대. 배포 구성 SSOT = [backend/config/deployments/](../backend/config/deployments/). 단일 머신 모드(`--host mock`)는 전 모듈 + mock driver.

| 머신 | host | IP | OS | modules |
| --- | --- | --- | --- | --- |
| PC (개발기) | `pc` | (LAN 동적) | Windows 11 | camera_decoded / calibration / scene3d / scan / waypoint / detector / llm / task / pick_and_place / bridge |
| hori1 | `pi_hori1` | 192.168.0.101 | Ubuntu 22.04 | motor / motion (so101_6dof_0) |
| hori2 | `pi_hori2` | 192.168.0.102 | Ubuntu 22.04 | camera (so101_6dof_0, D405) |
| hori3 | `pi_hori3` | 192.168.0.103 | Ubuntu 22.04 | motor / motion / camera (omx_f_0, USB UVC) |

분배의 핵심 이유:

- (a) **제어 루프 로컬화** — motion 의 고빈도 관절 명령 스트림이 네트워크를 안 넘게 motor + motion 은 항상 같은 머신.
- (b) **USB 대역폭 경합 해소** — D405 와 모터 컨트롤러가 한 USB 컨트롤러를 공유하지 않음 (so101 은 motor=hori1 / camera=hori2 분리).
- (c) **무거운 연산은 PC** — GDINO/SAM2, Open3D (ICP/TSDF), LLM, bridge.

### Pi 설정 / 실행

- Ubuntu 22.04 aarch64 + uv (3대 공통). 의존성 = host 별 dependency group ([backend/pyproject.toml](../backend/pyproject.toml)).
- **특수 처리 1개**: `pyrealsense2` (hori2 만) — aarch64 wheel 없음 → 소스 빌드, 절차 = [hardware.md](hardware.md).

```bash
# hori1 (so101 motor/motion)          # hori3 는 pi-hori3 로 동형
cd backend
uv sync --no-default-groups --group pi-hori1
uv run --no-sync python -m apps.main --host pi_hori1

# hori2 (so101 D405 — pyrealsense2 빌드/설치 선행)
uv sync --no-default-groups --group pi-hori2 --no-install-package pyrealsense2
uv pip install ~/pyrealsense2-*.whl
uv run --no-sync python -m apps.main --host pi_hori2
```

`--host` 는 필수 인자 — [apps/main.py](../backend/apps/main.py) 가 `config/deployments/{host}.yaml` 로드 (hostname 자동 감지 없음).

### 네트워크

- Zenoh **peer 모드** — 같은 LAN + 멀티캐스트면 자동 발견 (deployment yaml `zenoh.scouting.multicast`).
- 멀티캐스트 막힌 환경이면 `zenoh.connect` 에 endpoint 명시 (`tcp/<ip>:7447`), 방화벽 고정용 `zenoh.listen` 도 yaml 에 정의 가능.

> ⚠️ 같은 LAN 에 실 robot Pi backend 가 떠 있으면 PC 의 pytest/sim publish 가 **실 모터를 움직일 수 있음** — motion 관련 테스트 전 Pi 상태 확인.

---

## 2. OMX_F (omx_f_0)

OpenMANIPULATOR-X 커스텀 변형 — **5DOF arm** (ID 1~5) + 그리퍼 (ID 6, motors.yaml `kind: gripper`, IK/FK 제외).

### 컨트롤러 — OpenRB-150 (U2D2 호환 모드)

- SAMD21 기반. ROBOTIS "USB to Dynamixel" 스케치로 U2D2 호환 릴레이.
- USB **CDC-ACM** → `/dev/ttyACM0` (FTDI `/dev/ttyUSB*` 아님). Windows `COM6`.

### 모터

| Joint | Model | 정격 전압 | 비고 |
| --- | --- | --- | --- |
| 1 (base) / 2 (shoulder) / 3 (elbow) | XL430-W250 | 10.0~14.8V (12V 권장) | J2/J3 중력 부하 큼 |
| 4 (wrist pitch) / 5 (wrist roll) / 6 (gripper) | XL330-M288 | 3.7~6.0V (5V 권장) | |

raw `0..4095`, 중심 `2048`(=0°). `reverse`/`limit` 은 motors.yaml, 단위 변환 = [backend/modules/motion/units.py](../backend/modules/motion/units.py).

### 전원 토폴로지

```
[메인 PSU 11V] ──── OpenRB-150 (배럴잭)
                      ├─ 직접 분기 ──── XL430 체인 (J1-3) @ 11V   (정격 하한 근처 — 마진 작음)
                      └─ XL4015 강압 (CV/CC 5A) ──── XL330 체인 (J4-6) @ 5V
```

- 데이지 체인 2그룹이지만 **TTL 데이터는 한 버스 공유** (ID 1~6 같은 패킷 버스).
- **XL430 그룹이 정격 하한 근처 → J2/J3 자세 의존 sag** → 캘의 sag 모델이 흡수. 12V 승압 실험은 캘 결과 미달로 11V 복귀 (커밋 `adec924`).

---

## 3. SO-101 6DOF (so101_6dof_0) — 구축 기록

SO-101 follower (원본 5DOF+그리퍼) 에 **ts_flake wrist yaw mod** 로 1축 추가 = 6DOF + 그리퍼 (모터 7개). Feetech 버스 + Waveshare driver — OMX 의 Dynamixel 과 SDK/프로토콜이 완전히 다름 (backend 는 driver adapter 로 분기).

### 모터 배치 (7개)

| 위치 | 모터 | 출처 |
| --- | --- | --- |
| M1 base yaw | STS3215 | 키트 |
| **M2 shoulder** | **STS3250 (50kg·cm)** ⭐ | 별도 구매 (WowRobo C002) |
| M3 elbow / M4 wrist pitch / **M5 wrist yaw (mod 신설)** / M6 wrist roll / M7 gripper | STS3215 | 키트 |

```
base_yaw → shoulder → elbow → wrist_pitch → [wrist_yaw NEW] → wrist_roll → gripper
   J1        J2         J3        J4             J5               J6         J7
```

### 왜 shoulder 만 STS3250 인가 (하중 분석 요지)

full extension 정적 토크: **shoulder ~27-30 kg·cm ≈ STS3215(30kg·cm) 정격 한계** ⚠️ / elbow 50% / wrist 여유 → 병목 = shoulder. 어차피 mod 에 모터 1개 추가가 필요 → $30 차액으로 STS3250(50kg·cm, 67% margin) 을 shoulder 에 박고 키트 STS3215 를 wrist yaw 로 재배치 — sag 감소 + payload 회복 동시 해결.

- **기어비 매칭 필수** ⚠️: 키트 STS3215 = 1/345 (C018) → STS3250 도 **1/345 (C002)**. 다른 기어비면 그 모터만 raw↔rad 계수가 달라져 모터별 분기 필요. (기각: C044=1/191 leader용, C001=7.4V)
- drop-in 호환: 외형 45.2×24.7×35mm 동일, TTL/12V/커넥터 동일, LeRobot 공식이 sts3215+sts3250 mixed 패턴 지원 명시.
- 운영 가이드: 팔 수평 장시간 펼침 금지(발열), 무거운 건 elbow 굽혀 들기. 잔여 sag 는 캘 sag 모델이 흡수.

### Feetech provisioning (Dynamixel 과 갈리는 두 지점)

- **ID 굽기**: 출하 시 전부 ID=1 → 한 개씩 연결해 부여 (FD GUI = Feetech 판 Wizard, 1회성이라 권장). 배치 = 위 표 M1..M7 = ID 1..7.
- **PID = EEPROM** (Dynamixel 은 RAM): 전원 사이클에도 유지, 8-bit, 동적 게인 스케줄링 비권장 (EEPROM wear). backend 동기화는 **read-first-then-write** (일치하면 write 0회) + EEPROM lock register 풀고 잠그기. "default 게인 → sag/진동 보고 한 번 튜닝 → 끝" 이 STS 답.

### 개조/CAD 자산 (외부)

| 자산 | 출처 |
| --- | --- |
| wrist yaw mod (3D 부품 2개, ~3h 출력) | [MakerWorld ts_flake SO101 6DoF](https://makerworld.com/ko/models/1913316-so101-arm-wrist-yaw-6dof) |
| mod URDF / onshape config / 메시 | [github.com/ts-flake/d2lrobot](https://github.com/ts-flake/d2lrobot) — mesh 경로가 절대경로 주의, 카메라 mesh 는 UVC 라 D405 로 교체 필요 |
| 공식 URDF(5DOF) / STL / D405 마운트 | [github.com/TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) |

**D405 wrist mount** = 공식 `Optional/Wrist_Cam_Mount_RealSense_D405/` STL 그대로 (DIN 912 M3x6/8 ×2). mod 의 wrist yaw 는 wrist roll 앞단 삽입이라 **roll 부품은 불변 → 마운트 호환 충돌 없음**.

구매 기록: Amazon B0GRPJ2Q8F (follower 풀번들) + WowRobo STS3250 C002 (2026-05-29). 상세 예산/대안 비교는 git history 의 `so101_6dof_plan.md`.

---

## 4. 카메라

| Robot | 카메라 | 사양 | 비고 |
| --- | --- | --- | --- |
| SO-101 | Intel RealSense D405 (wrist) | RGBD, factory intrinsic seed (color 1280×720, fx≈649, fy≈648) | 권장 작동거리 10–50cm, USB-C. hori2 |
| OMX | 720P USB 2.0 UVC | 1280×720 / DFOV 120° 광각 | 표준 UVC. hori3 |

- swap 완료 (2026-06): D405 → SO-101, OMX 는 UVC 다운그레이드 — 6DOF 정밀 manipulation 에 RGBD 가 더 가치라는 design intent. robots.yaml `camera_backend` 가 SSOT.
- 캘 의미: **D405** = factory intrinsic seed, hand_eye 만 캘 / **UVC 광각** = factory intrinsic 없음 + barrel distortion 큼 → **intrinsic 캘이 first step** (plumb_bob 5-param 부족하면 rational/fisheye 검토).
- σ floor: OMX+D405 시절 0.65°/7.94mm, SO-101+D405 effective 0.801°/7.53mm ([calibration.md](calibration.md)).

---

## 5. 작업대

- 책상 **가로 55 × 세로 34 cm** (보드 + 로봇 + 작업영역 공유). 로봇 base 배치는 robots.yaml `base_pose` (so101 x=0.4m).
- OMX reach: stretched 500mm, 자세 다양성 영역 ≈ 350-400mm sphere.
- 캘 보드 7×5 / 25mm 선정은 이 작업대 + reach 압박 고려 ([calibration.md](calibration.md)).


---
---

# 부록 — 통합 원문 (2026-07-11 문서 다이어트)

> 아래 문서들을 본 문서 부록으로 병합 (원문 그대로):
> - `hardware.md`


---
---

<!-- ═══════════ [통합 원문] hardware.md ═══════════ -->

# pyrealsense2를 Pi에서 직접 빌드한다는 게 무슨 뜻인가

> 환경: Raspberry Pi 4B+ 4GB / Ubuntu 22.04 aarch64 / uv + Python 3.11
> 목적: 공부용. 각 단계가 "왜" 필요한지 + "무엇이" 일어나는지 깊게 이해.

---

## 0. 큰 그림부터 — "왜 그냥 pip install이 안 되나"

평소 우리가 쓰는 `pip install numpy` 같은 명령은 사실 **이미 컴파일된 결과물**을 받아오는 것뿐이다.

```
PyPI 서버에 있는 것:
  numpy-1.26.0-cp311-cp311-win_amd64.whl         ← Windows x86_64 + Python 3.11용
  numpy-1.26.0-cp311-cp311-linux_x86_64.whl      ← Linux x86_64 + Python 3.11용
  numpy-1.26.0-cp311-cp311-manylinux_aarch64.whl ← Linux ARM64 + Python 3.11용
  ...
```

`pip`은 내 환경(OS + CPU 아키텍처 + Python 버전)에 맞는 wheel을 골라서 다운로드한다. **C 코드는 이미 컴파일되어 있어서** 받자마자 import 가능한 거다.

그런데 `pyrealsense2`는 Intel이 **x86_64 wheel만 PyPI에 올려놨다.** Pi 4의 CPU는 ARM64(=aarch64)라서 받을 wheel이 없다. 그래서:

> **"소스코드를 받아서, 내 Pi에서 직접 컴파일해서, 내 Pi용 .so 파일을 만든다."**

이게 이 작업의 본질이다. 일반 사용자가 PyPI 서버 역할을 한 번 대신 해주는 셈.

---

## 1. 빌드 파이프라인 — 무엇이 무엇을 만드는가

작업 순서가 왜 이렇게 복잡한지 이해하려면, **결과물의 의존 관계**를 먼저 봐야 한다.

```
[최종 목표: import pyrealsense2 가 동작]
        ↑
[pyrealsense2.cpython-311-aarch64-linux-gnu.so]  ← Python에서 import할 파일
        ↑ 이걸 빌드하려면
[librealsense2.so]                                ← C++ 라이브러리
        ↑ 이걸 빌드하려면
[protoc, libprotobuf.so]                          ← Google protobuf (데이터 직렬화)
        ↑ 이걸 빌드하려면
[cmake, clang, libusb-1.0-dev, libtbb-dev, ...]   ← 빌드 도구 + 시스템 라이브러리
```

각 단계는 **아랫단이 끝나야 윗단을 시작할 수 있다.** 그래서 Step 1 → Step 8까지가 한 줄로 늘어선 것.

용어부터 정리:

| 용어 | 한 줄 설명 |
|---|---|
| **소스코드** | 사람이 읽을 수 있는 C/C++/Python 텍스트 (`.cpp`, `.h`) |
| **컴파일러** | 소스코드 → 기계어로 번역하는 프로그램 (`gcc`, `clang`) |
| **`.so` 파일** | Shared Object. Linux의 공유 라이브러리 (Windows의 `.dll` 같은 것) |
| **링킹(linking)** | 여러 `.o`(object 파일)을 하나의 실행파일/라이브러리로 합치는 과정 |
| **cmake** | 빌드 설정 도구. `Makefile`을 자동 생성해줌 |
| **make** | `Makefile`을 보고 실제 컴파일을 수행하는 도구 |
| **wheel (`.whl`)** | Python 패키지 배포 포맷. 사실은 이름 규칙이 있는 `.zip` |
| **pybind11** | C++ 클래스/함수를 Python에서 쓸 수 있게 변환해주는 라이브러리 |

---

## 2. Step 1 — 시스템 의존성 설치 (= "빌드 도구 갖추기")

```bash
sudo apt-get update && sudo apt-get dist-upgrade -y
sudo apt-get install -y automake libtool cmake libusb-1.0-0-dev libx11-dev \
  xorg-dev libglu1-mesa-dev libssl-dev clang llvm libatlas-base-dev \
  python3-opencv libtbb-dev
```

이 단계는 **요리 시작 전 도구 + 식재료 준비**라고 보면 된다.

### 두 종류로 나뉨

**(A) 빌드 도구 — "조리 도구"**

| 패키지 | 비유 | 실제 역할 |
|---|---|---|
| `cmake` | 레시피 → 조리 순서표 변환기 | 프로젝트의 `CMakeLists.txt`를 보고 `Makefile`을 생성 |
| `automake`, `libtool` | 오래된 스타일 조리 순서표 도구 | protobuf 같은 옛날 프로젝트에서 사용 |
| `clang`, `llvm` | 고급 조리 도구 (gcc 대신) | C/C++ 컴파일러. Intel이 권장 |

**(B) 시스템 라이브러리 — "식재료"**

| 패키지 | 무엇? | 왜 librealsense가 필요로 하나 |
|---|---|---|
| `libusb-1.0-0-dev` | USB 통신 표준 라이브러리 | RealSense 카메라가 USB로 연결되니까. **이게 없으면 카메라랑 말을 못 함** |
| `libssl-dev` | SSL/TLS (HTTPS 같은 것) | librealsense의 펌웨어 업데이트 기능 등에서 사용 |
| `libtbb-dev` | Intel TBB (Threading Building Blocks) | C++ 병렬처리. 깊이 영상 처리에 멀티코어 활용 |
| `libatlas-base-dev` | BLAS 수학 라이브러리 | 행렬 연산 (point cloud 변환 등) |
| `libx11-dev`, `xorg-dev`, `libglu1-mesa-dev` | X11 + OpenGL | 예제 프로그램(`realsense-viewer`)이 GUI 띄울 때 |

### `-dev` 접미사가 붙은 이유

`libusb-1.0-0` (런타임 라이브러리)와 `libusb-1.0-0-**dev**` (개발용 헤더)는 다른 패키지다.

- 그냥 라이브러리: `.so` 파일만 있음 → 이미 컴파일된 프로그램이 실행될 때 사용
- `-dev` 패키지: `.so` + **헤더 파일(`.h`)** → 새 프로그램을 컴파일할 때 필요

우리는 **컴파일을 할 거니까 `-dev` 버전이 필요**.

### 원본 가이드 vs 우리 환경

원본은 Raspberry Pi OS 32-bit(armhf) 기준이라 `libtbb-dev_2018U2_armhf.deb`를 수동으로 받아야 했다. 우리는 Ubuntu 22.04 aarch64라 apt 저장소에 이미 정식 패키지가 있어서 `sudo apt install libtbb-dev` 한 줄이면 끝.

---

## 3. Step 2 — Swap 공간 확보 (= "RAM 부족 대비책")

```bash
sudo swapoff -a
sudo fallocate -l 2G /swapfile     # 디스크에 2GB 빈 파일 생성
sudo chmod 600 /swapfile
sudo mkswap /swapfile              # 그 파일을 swap 포맷으로 마킹
sudo swapon /swapfile              # 시스템에 "이걸 swap으로 써라"
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Swap이란?

**"RAM이 부족할 때 디스크를 RAM처럼 빌려 쓰는 것."**

- RAM(4GB) ━━━ 빠름, 비쌈, 휘발성
- Swap(디스크 일부) ━━━ 느림, 큼, 비휘발성

평소엔 RAM만 쓰지만, RAM이 꽉 차면 OS가 "당장 안 쓰는 메모리 페이지"를 디스크의 swap 영역으로 잠깐 옮기고, 그 자리에 새 데이터를 올린다.

### 왜 빌드 중에 필요한가

`make -j4`는 4개 CPU 코어로 **동시에** 4개의 파일을 컴파일한다. C++ 컴파일러는 한 파일 컴파일하는 데 메모리를 꽤 먹어서:

```
컴파일러 4개 × 각 ~800MB = 3.2GB
+ OS, 데스크톱 등 = 4GB RAM 거의 꽉 참
+ 큰 .cpp 파일 만나면 한 컴파일러가 1.5GB 요구 → OOM 발생
```

OOM(Out Of Memory) 발생하면 리눅스가 `OOM Killer`로 프로세스를 강제 종료시킨다. 그러면 빌드가 갑자기 `make: *** Error 137` 같은 걸로 죽는다. 2GB swap을 추가하면 그 순간 디스크로 잠깐 흘려보내서 살아남게 해주는 거다.

### `/etc/fstab` 등록

`/etc/fstab`은 **부팅 시 자동으로 마운트할 것들의 목록**이다. swap을 여기 등록하면 재부팅 후에도 swap이 살아있다. 등록 안 하면 재부팅 후 `swapon`을 다시 해야 한다.

> **주의: SD카드는 쓰기 횟수 수명이 짧음.** Swap을 너무 자주 쓰면 SD카드 수명이 줄어든다. 빌드 후엔 swap 비활성화 또는 작게 줄여도 OK.

---

## 4. Step 3 — udev rules (= "USB 카메라 접근 권한 열기")

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo su -c "udevadm control --reload-rules && udevadm trigger"
```

### udev가 뭐길래?

Linux에서 **USB나 외부 장치가 꽂힐 때 처리하는 시스템**이 udev다. USB를 꽂으면:

```
1. 커널이 "USB 장치 발견!" 이벤트 발행
2. udev 데몬이 이벤트 받음
3. /etc/udev/rules.d/ 의 규칙들 순차 검사
4. 매칭되는 규칙에 따라 /dev/ 아래에 디바이스 파일 생성 + 권한 설정
```

### 기본 동작의 문제

Linux는 안전상 **USB 장치는 root만 접근 가능**하게 만든다. 그래서 일반 사용자(`hori2`)가 `pyrealsense2.pipeline().start()` 하면 "권한 없음" 에러가 난다.

### 99-realsense-libusb.rules가 하는 일

이 파일에는 RealSense 카메라들의 **USB 식별자(VID:PID)** 가 적혀 있다:

```
# 대충 이런 식
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5b", MODE:="0666"
                                  ↑Intel    ↑D405 모델          ↑모든 사용자 RW
```

D405(`0b5b`), D435(`0b07`), D455(`0b5c`) 등이 다 들어있다. 이 규칙을 등록하면 RealSense 카메라가 꽂힐 때 udev가 알아서 권한을 `0666`(모두 읽기쓰기)로 열어준다.

### 명령어 의미

- `udevadm control --reload-rules` — udev에게 "규칙 파일 다시 읽어"
- `udevadm trigger` — "지금 이미 꽂혀있는 장치들에도 새 규칙 적용해줘"

> **Tip:** `lsusb`로 카메라가 보이는지, `ls -l /dev/bus/usb/<bus>/<dev>`로 권한이 `crw-rw-rw-`인지 확인 가능.

---

## 5. Step 4 — 환경변수 설정 (= "OS에게 어디서 찾으라고 알려주기")

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION_VERSION=3
export PYTHONPATH=$PYTHONPATH:/usr/local/lib:/home/hori2/librealsense/build/Release
```

### `LD_LIBRARY_PATH` — 동적 라이브러리 검색 경로

프로그램이 실행될 때 `.so` 파일이 필요하면 OS가 **정해진 경로 목록**을 순서대로 뒤진다:

```
1. LD_LIBRARY_PATH 환경변수에 있는 경로들
2. /etc/ld.so.cache (ldconfig가 만든 캐시)
3. /lib, /usr/lib
```

`sudo make install`로 librealsense를 `/usr/local/lib/`에 깔았으니까, 그 경로를 검색 목록에 넣어주는 거다.

> **비유:** "라면 어디 있어?" 라고 물었을 때 부엌 → 거실 → 창고 순으로 찾는 것처럼, OS도 정해진 순서로 라이브러리를 찾는다.

### `PYTHONPATH` — Python 모듈 검색 경로

`import pyrealsense2` 라고 하면 Python은:

```
1. 현재 디렉토리
2. PYTHONPATH의 경로들
3. site-packages (pip install된 것들)
```

순서로 모듈을 찾는다. 우리가 빌드한 `pyrealsense2.cpython-311-aarch64-linux-gnu.so`는 `build/Release/`에 있으니, 그 경로를 PYTHONPATH에 추가하면 import 가능.

> **이게 임시방편인 이유:** Step 8에서 wheel로 패키징하면 site-packages에 정식 설치되니까 PYTHONPATH 안 건드려도 됨. 환경변수 의존은 "어디서 실행하느냐"에 따라 깨지기 쉬워서, **최종 단계에서 wheel로 정리**한다.

### `PROTOCOL_BUFFERS_*` 변수

protobuf는 Python 구현체(느림)와 C++ 구현체(빠름) 두 가지가 있다. C++ 구현을 쓰라고 명시해서 성능 확보.

### 빌드 결과물 경로 주의

원본 가이드 경로 `build/wrappers/python`은 예전 버전 구조. 실제 빌드 결과는 **`build/Release/`** 에 생성됨 → PYTHONPATH도 이 경로로 설정.

---

## 6. Step 5 — protobuf 빌드 (= "데이터 형식 사전 만들기")

```bash
cd ~
git clone --depth=1 -b v3.10.0 https://github.com/google/protobuf.git
cd protobuf
./autogen.sh && ./configure
make -j4
sudo make install
sudo ldconfig
```

### protobuf가 뭔가?

**Protocol Buffers** — Google이 만든 **데이터 직렬화 라이브러리**. JSON과 비슷한데:

| | JSON | protobuf |
|---|---|---|
| 형식 | 텍스트 | 바이너리 |
| 크기 | 큼 | 작음 (3~10배) |
| 속도 | 느림 | 빠름 |
| 스키마 | 없음 (자유) | `.proto` 파일로 미리 정의 |

librealsense는 카메라 펌웨어와 통신할 때 protobuf를 사용한다.

### 왜 특정 버전(v3.10.0)?

librealsense가 v3.10.0 API에 의존해서 만들어졌다. v3.20을 쓰면 함수 시그니처가 바뀌어서 컴파일이 안 된다. **버전 호환성** 때문에 정확히 그 버전을 박은 것.

### `./autogen.sh && ./configure && make` — GNU Autotools 방식

cmake보다 오래된 빌드 시스템이다. 순서대로:

1. **`./autogen.sh`** — 개발자가 작성한 템플릿(`configure.ac`)에서 `configure` 스크립트를 생성
2. **`./configure`** — 시스템을 검사 (어떤 컴파일러? 어떤 라이브러리 있나?) 후 `Makefile` 생성
3. **`make`** — 실제 컴파일
4. **`sudo make install`** — 결과물을 `/usr/local/bin`, `/usr/local/lib` 등에 복사

### `sudo ldconfig`의 의미

`/usr/local/lib`에 새 `.so` 파일을 깔아도 OS는 모른다. `ldconfig`는 시스템 라이브러리 캐시(`/etc/ld.so.cache`)를 다시 만들어서 "여기 새 라이브러리 생겼어!"를 등록해주는 명령이다.

### "Python 바인딩 빌드 실패 — 무시해도 됨"의 의미

protobuf 자체는 C++ 라이브러리다. 추가로 "Python에서도 protobuf 메시지를 쓰고 싶다"라는 사람들을 위한 Python 바인딩이 따로 있다. **우리한테 필요한 건 C++ 라이브러리**(librealsense가 이걸 link함)이지 Python 바인딩이 아니다. 그래서 Python 쪽이 깨져도 무관.

`protoc --version`이 `libprotoc 3.10.0`으로 뜨면 C++ 부분은 잘 깔린 것.

---

## 7. Step 6 — librealsense C++ 빌드 (= "본진")

```bash
cd ~/librealsense
mkdir build && cd build

export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

cmake .. \
  -DBUILD_EXAMPLES=true \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_LIBUVC=true \
  -DOTHER_LIBS="-latomic"

make -j4
sudo make install
```

### `mkdir build && cd build` — out-of-source build

소스 트리는 깨끗하게 두고, 빌드 산출물은 별도 `build/` 폴더에 만드는 관례. 빌드 망쳤을 때 `rm -rf build`로 통째로 날리면 깨끗해져서 좋다.

### `CC`, `CXX` 환경변수

cmake가 어떤 컴파일러를 쓸지 결정할 때 이 환경변수를 본다.

- `CC` = C 컴파일러
- `CXX` = C++ 컴파일러

기본은 gcc인데, librealsense는 일부 코드가 gcc의 strict 모드와 충돌 → Intel이 clang을 권장. 그래서 강제로 clang 지정.

### cmake 플래그 한 줄씩

| 플래그 | 의미 | 왜 |
|---|---|---|
| `-DBUILD_EXAMPLES=true` | `realsense-viewer` 같은 예제 같이 빌드 | 카메라 동작 확인용 GUI |
| `-DCMAKE_BUILD_TYPE=Release` | 최적화 ON (`-O3`), 디버그 심볼 제거 | 빠른 실행. `Debug`보다 ~5배 빠름 |
| `-DFORCE_LIBUVC=true` | **이게 라즈베리파이 핵심** | 아래 별도 설명 |
| `-DOTHER_LIBS="-latomic"` | libatomic을 같이 link | ARM에선 64-bit atomic 연산이 별도 라이브러리 |

### `FORCE_LIBUVC=true` 가 왜 필수인가

UVC = USB Video Class. 웹캠, 스마트폰 카메라 등이 따르는 표준 USB 프로토콜이다.

Linux는 UVC 드라이버가 **커널 레벨**에 있다(`uvcvideo` 모듈). librealsense는 기본적으로 이 커널 드라이버를 패치해서 RealSense 전용 기능(컨트롤 명령, 깊이 스트림 등)을 추가하는 방식으로 동작한다.

**문제:** Raspberry Pi의 커널은 ARM용으로 커스텀 빌드된 거라 패치 적용이 까다롭거나 불가능.

**해결책:** `FORCE_LIBUVC=true` → 커널 드라이버 안 건드리고 **유저스페이스에서 USB를 직접 다루는** libuvc 라이브러리로 대체. 약간 느리지만 잘 동작.

> **비유:** 보통은 차의 ECU 펌웨어를 수정해서 성능을 올리는데(커널 패치), Pi에선 그게 안 되니까 별도 외장 컴퓨터를 달아서(libuvc) 제어하는 식.

### `-latomic` 이 왜 필요한가

C++에서 `std::atomic<int64_t>`를 쓸 때, **64비트 atomic 연산**은 ARM에선 단일 명령어로 안 됨 → 소프트웨어로 구현된 함수 호출이 필요 → 그 함수들이 `libatomic.so`에 있다. 그래서 link 시 명시 필요.

x86_64는 CPU가 64-bit atomic 명령어를 네이티브로 지원해서 안 붙여도 됨.

### `sudo make install` 결과

```
/usr/local/lib/librealsense2.so          ← 메인 C++ 라이브러리
/usr/local/lib/librealsense2-gl.so       ← OpenGL 확장
/usr/local/include/librealsense2/        ← 헤더 파일들
/usr/local/bin/rs-enumerate-devices      ← CLI 도구
/usr/local/bin/realsense-viewer          ← GUI 도구
```

이 시점에 `realsense-viewer`를 실행하면 카메라가 동작한다 (Python 빌드 전이지만 C++만으로도 카메라는 됨).

빌드 끝나면 `unset CC / unset CXX`로 환경변수 원복해서 시스템 기본 컴파일러(gcc)를 되돌리는 게 좋다.

---

## 8. Step 7 — pyrealsense2 Python 바인딩 빌드

```bash
cd ~/librealsense/build

cmake .. \
  -DBUILD_PYTHON_BINDINGS=bool:true \
  -DPython_ROOT_DIR=<uv Python 경로> \
  -DPython_INCLUDE_DIR=<uv Python include 경로> \
  -DPython_LIBRARY=<uv Python lib 경로>

make -j4
sudo make install
```

### Python 바인딩이 뭐길래?

C++ 라이브러리를 Python에서 쓸 수 있게 해주는 **번역 레이어**다.

```
[Python 코드]
import pyrealsense2 as rs
pipe = rs.pipeline()
       ↓ 이때 일어나는 일
[pyrealsense2.cpython-311-aarch64-linux-gnu.so]
       ↓ 이 .so 안에 pybind11이 생성한 Python ↔ C++ 변환 코드
[librealsense2.so의 rs2_pipeline 객체 생성 함수 호출]
```

**pybind11**이라는 C++ 라이브러리가 이걸 도와준다. 헤더만 include하고 매크로 몇 줄 적으면 C++ 클래스가 Python 클래스로 자동 변환된다.

### `cmake ..` 를 또 부르는 이유

같은 `build/` 폴더지만 **다른 플래그**(`BUILD_PYTHON_BINDINGS=true`)로 재설정하는 거다. cmake는 캐시(`CMakeCache.txt`)에 이전 설정을 저장하고, 변경된 부분만 다시 처리한다.

### uv Python 경로 명시가 왜 필요한가

cmake는 시스템에 깔린 Python을 자동 탐색하는 모듈(`FindPython`)이 있는데, 보통 다음을 찾는다:

```
/usr/bin/python3
/usr/lib/python3.11/...
/usr/include/python3.11/Python.h
```

그런데 **uv는 자기만의 Python을 따로 설치**한다:

```
/home/hori2/.local/share/uv/python/cpython-3.11.15-linux-aarch64-gnu/
  ├── bin/python3.11
  ├── include/python3.11/Python.h
  └── lib/libpython3.11.so
```

cmake가 이 경로를 모르니까 `Could NOT find Python` 에러가 난다. **이게 우리 환경 특유의 함정**이다. 일반 시스템 Python 쓰면 발생 안 함.

해결: 세 가지 변수를 명시:
- `Python_ROOT_DIR` — uv Python의 루트 디렉토리
- `Python_INCLUDE_DIR` — `Python.h`가 있는 곳 (C++가 Python API 쓰려면 필요)
- `Python_LIBRARY` — `libpython3.11.so` 위치 (link 시 필요)

### 결과물

```
~/librealsense/build/Release/
├── pyrealsense2.cpython-311-aarch64-linux-gnu.so   ← 이게 진짜 import 대상
├── pyrsutils.cpython-311-aarch64-linux-gnu.so
├── librealsense2.so                                  ← 심볼릭 링크 또는 복사본
└── librealsense2-gl.so
```

### `.so` 파일명 해독

```
pyrealsense2 . cpython-311 - aarch64 - linux - gnu . so
   ↑모듈명      ↑Python 구현+버전  ↑CPU    ↑OS     ↑ABI    ↑확장자
```

이 이름이 곧 **이 파일이 어디서 동작 가능한지의 명세**다. Python이 import할 때 이 이름을 보고 "내 환경(CPython 3.11, ARM64, Linux GNU)이랑 맞네" 확인 후 로드.

---

## 9. Step 8 — Wheel 패키징 (= "내가 만든 빌드 산출물을 정식 Python 패키지로 만들기")

### Wheel 파일이 뭔가?

**Python 패키지 배포의 표준 포맷.** 알고 보면 그냥 **특별한 이름 규칙을 가진 zip 파일**이다.

```bash
unzip pyrealsense2-2.55.1-cp311-cp311-linux_aarch64.whl
# 안에 뭐가 있나
├── pyrealsense2/
│   ├── __init__.py
│   ├── pyrealsense2.cpython-311-aarch64-linux-gnu.so
│   └── ...
└── pyrealsense2-2.55.1.dist-info/
    ├── METADATA       ← 패키지 정보 (이름, 버전, 의존성)
    ├── WHEEL          ← wheel 포맷 버전
    └── RECORD         ← 파일 목록 + 체크섬
```

### 파일명 규칙

```
pyrealsense2 - 2.55.1 - cp311 - cp311 - linux_aarch64 .whl
    ↑이름      ↑버전   ↑Python ↑ABI   ↑플랫폼
                       태그    태그
```

`pip`(또는 `uv`)이 wheel을 받을 때 **이 파일명만 보고** "내 환경에 맞나" 판단한다. 이름이 안 맞으면 설치 거부.

### 왜 wheel을 만드는 게 좋은가

지금까지 우리가 한 건 `.so` 파일을 `~/librealsense/build/Release/`에 만들어둔 것뿐이다. 이대로는:

| 문제 | 환경변수 방식 (현재) | wheel 방식 (Step 8) |
|---|---|---|
| import 되나? | `PYTHONPATH` 설정해야 됨 | 그냥 `import` 됨 |
| `uv run`에서? | PYTHONPATH 전파 까다로움 | `uv pip install`만 하면 끝 |
| 다른 Pi로 옮기려면? | 빌드 다시? | wheel 파일 하나만 복사하면 됨 |
| `pyproject.toml` 관리? | 안 됨 (수동 의존성) | dependencies에 등록 가능 |

### CLAUDE.md에 나오는 흐름과 연결

CLAUDE.md에 이런 줄이 있다:

```
uv sync --only-group pi-camera --no-install-package pyrealsense2
uv pip install ./pyrealsense2-*.whl
```

- `--no-install-package pyrealsense2` — PyPI에서 pyrealsense2 받으려 하지 마 (어차피 ARM64 wheel 없어서 실패함)
- `uv pip install ./pyrealsense2-*.whl` — 내가 빌드한 wheel을 별도로 설치

이렇게 두 단계로 분리하는 게 **재현 가능한 셋업**의 핵심.

### Wheel을 어떻게 만드는가 (간단 버전)

1. 적절한 디렉토리 구조 만들기:
   ```
   pyrealsense2_pkg/
   ├── pyrealsense2/
   │   ├── __init__.py            # from .pyrealsense2 import *
   │   └── pyrealsense2.cpython-311-aarch64-linux-gnu.so
   ├── pyproject.toml             # name, version, etc.
   └── ...
   ```

2. `pip wheel .` 또는 `python -m build` 실행 → `.whl` 생성

3. 생성된 wheel을 다른 Pi에도 복사해서 `uv pip install ./xxx.whl`

(또는 더 빠르게: 이미 만들어진 `build/Release/` 내용을 직접 zip으로 묶고 wheel 메타데이터 첨부)

---

## 10. 전체 흐름 한눈에 다시 보기

```
┌─────────────────────────────────────────────────────────────────┐
│ STEP 1: 시스템 도구 + 라이브러리 설치 (apt)                       │
│   → cmake, clang, libusb, libtbb 등 "재료와 도구"                │
└─────────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 2: Swap 2GB 추가                                            │
│   → RAM 부족 대비. make -j4가 OOM으로 죽지 않게                  │
└─────────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 3: udev rules 적용                                          │
│   → USB 카메라를 일반 사용자가 접근 가능하게                     │
└─────────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 4: 환경변수 (LD_LIBRARY_PATH, PYTHONPATH)                   │
│   → 빌드한 .so를 OS와 Python이 찾을 수 있게                      │
│   (Step 8에서 wheel 패키징하면 사실상 불필요해짐)                │
└─────────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 5: protobuf v3.10.0 빌드                                    │
│   → librealsense가 의존하는 데이터 직렬화 라이브러리             │
└─────────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 6: librealsense C++ 빌드 (FORCE_LIBUVC=true, clang)         │
│   → librealsense2.so 생성. 이 시점에 realsense-viewer 동작       │
└─────────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 7: pyrealsense2 Python 바인딩 빌드                          │
│   → pyrealsense2.cpython-311-aarch64-linux-gnu.so 생성           │
│   → 이 시점에 PYTHONPATH 설정하면 import 가능                    │
└─────────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 8: wheel로 패키징 → uv pip install                          │
│   → 정식 패키지로 설치. PYTHONPATH 의존 제거                     │
└─────────────────────────────────────────────────────────────────┘
                          ↓
                  import pyrealsense2 ✓
```

---

## 11. 트러블슈팅 기록

| 증상 | 원인 | 해결 |
|---|---|---|
| `libtbb-dev_armhf.deb` 설치 불가 | 32-bit 패키지, aarch64 비호환 | `sudo apt install libtbb-dev` |
| `Python.h: No such file or directory` (protobuf) | protobuf Python 바인딩 빌드 실패 | C 라이브러리는 성공했으므로 무시 |
| `Could NOT find Python` (cmake) | uv Python이 비표준 경로에 있어 자동 탐색 실패 | `Python_ROOT_DIR`, `Python_INCLUDE_DIR`, `Python_LIBRARY` 명시 |
| `.so` 파일이 `build/wrappers/python`에 없음 | 최신 librealsense는 `build/Release/`에 생성 | PYTHONPATH 경로 수정 |
| `module has no attribute '__version__'` | pyrealsense2는 `__version__` 속성이 없음 | `dir(rs)` 로 정상 확인 — 정상 동작 |
| `make: *** Error 137` 같은 갑작스러운 빌드 종료 | OOM Killer가 컴파일러 죽임 | Swap 늘리기, `make -j2`로 병렬도 낮추기 |

---

## 12. 공부 포인트 — 이 작업에서 진짜로 배운 것

이 빌드 과정은 그냥 "라이브러리 깔기"가 아니라 **시스템 프로그래밍의 축소판**이다. 다음 개념들이 모두 나온다:

1. **컴파일 vs 인터프리트** — Python 모듈이라도 결국 C++ 컴파일이 필요할 수 있다
2. **아키텍처 호환성** — x86_64 wheel ≠ aarch64 wheel
3. **동적 라이브러리와 링킹** — `.so` 파일은 실행 시점에 로드된다 (`LD_LIBRARY_PATH`)
4. **빌드 시스템 계층** — `cmake → Makefile → make → gcc/clang`
5. **유저스페이스 vs 커널스페이스** — `FORCE_LIBUVC`가 왜 필요한지 (커널 드라이버 우회)
6. **udev와 디바이스 권한** — Linux가 USB 권한을 어떻게 다루는지
7. **패키지 포맷의 본질** — wheel이 그냥 zip이라는 사실
8. **빌드와 런타임의 분리** — `-dev` 패키지 vs 런타임 패키지

면접 질문에서 "라즈베리파이에서 라이브러리 어떻게 빌드했어요?" 라고 물으면 이 흐름을 줄줄 말할 수 있으면 된다고 보면 된다.
