# Hardware

OMX_F는 OpenMANIPULATOR-X(OMX)의 커스텀 변형. 운동학적으로 **5DOF arm** (모터 ID 1~5) + **그리퍼 1개** (ID 6, `core.common.GRIPPER_ID`로 IK/FK 대상에서 제외).

---

## 모터 컨트롤러 — OpenRB-150 (U2D2 호환 모드)

- SAMD21 기반, Arduino MKR Zero 호환 보드. **U2D2 정품(FTDI) 아님**.
- ROBOTIS의 "USB to Dynamixel" 예제 스케치를 올려 U2D2 호환 모드로 사용 (소프트 USB↔TTL 패킷 릴레이).
- USB **CDC-ACM** 클래스 → Pi에서 `/dev/ttyACM0`로 enumerate (FTDI 같은 `/dev/ttyUSB*` 아님).
- 기본 모터 포트: Windows `COM6` / Linux `/dev/ttyACM0` ([robot/config/motors.yaml](../robot/config/motors.yaml)).

---

## 모터 사양

| Joint             | Model      | 정격 전압 (operating) | 비고                     |
| ----------------- | ---------- | --------------------- | ------------------------ |
| 1 (base rotation) | XL430-W250 | 10.0~14.8V (12V 권장) |                          |
| 2 (shoulder)      | XL430-W250 | "                     | 중력 부하 큼             |
| 3 (elbow)         | XL430-W250 | "                     | 중력 부하 큼             |
| 4 (wrist pitch)   | XL330-M288 | 3.7~6.0V (5V 권장)    |                          |
| 5 (wrist roll)    | XL330-M288 | "                     |                          |
| 6 (gripper)       | XL330-M288 | "                     | `core.common.GRIPPER_ID` |

각 모터 raw는 `0..4095`, 중심 `2048`(=0°). [robot/config/motors.yaml](../robot/config/motors.yaml)의 각 모터에 `reverse` 플래그와 `limit.min/max` raw 클램프 (`rad_to_raw`가 강제). 단위 변환은 [backend/core/units.py](../backend/core/units.py).

---

## 전원 토폴로지

```
[메인 PSU 11V] ──── OpenRB-150 (배럴잭, 또한 통신/스케치)
                      │
                      ├─ 직접 분기 ──── XL430 체인 (joint 1, 2, 3) @ 11V
                      │                    (정격 10~14.8V — 하한 가까움, 마진 작음)
                      │
                      └─ XL4015 DC-DC 강압 모듈 (CV/CC 5A) ──── XL330 체인 (joint 4, 5, 6) @ 5V
                                                                  (정격 3.7~6V — 정중앙)
```

- 데이지 체인은 두 그룹으로 분리되어 있으나 **TTL 데이터 라인은 같은 버스를 공유** (Dynamixel half-duplex). 즉 ID 1~6이 모두 한 패킷 버스 위에 있음.
- Wizard에서 확인된 실측: XL430 그룹 10~11V, XL330 그룹 ~5V — 둘 다 정격 안.
- **XL430 그룹이 정격 하한 근처라 토크 마진이 작음**. joint 2/3에서 자세 의존적 sag 발생 → BA의 sag 모델이 이걸 흡수 ([docs/calibration_apply_flow.md § 3](calibration_apply_flow.md)).
- 12V 승압 실험은 2026-05 캘 결과 미달로 11V 복귀 (커밋 `adec924`).

---

## 카메라

> **현재 상태 (2026-06)**: D405 는 **OMX 에 부착**, SO-101 **미도착** (실물 없음). 아래 swap plan 은 SO-101 수령 후 적용. 캘리브레이션 σ floor (0.65°/7.94mm) 도 OMX+D405 조합 수치.

| Robot | 카메라 | 사양 | 비고 |
|---|---|---|---|
| OMX | 720P USB 2.0 UVC | 1280×720 / DFOV 120° (HFoV≈113° / VFoV≈81°) / 광각 | OMX 기본 동봉 카메라. Pi/Jetson/Win/Linux/Mac/Android 표준 UVC |
| SO-101 | Intel RealSense D405 | RGBD, factory intrinsic seed (color 1280×720, fx≈649, fy≈648, cx≈633, cy≈360) | 권장 작동거리 10–50cm. 단초점, USB-C |

**Swap plan** ([distributed_topology.md](distributed_topology.md)): SO-101 도착 시 D405 → SO-101 로 이관, OMX 는 위 USB UVC 카메라로 다운그레이드. SO-101 의 6DOF 정밀 manipulation 에 RGBD 가 더 가치라는 design intent.

캘리브레이션 의미 ([calibration_workflow.md](calibration_workflow.md)):
- **D405** — factory intrinsic seed 사용, hand_eye 만 캘 필요
- **USB UVC (광각)** — factory intrinsic 없음 + DFOV 120° barrel distortion 큼 → **intrinsic 재캘이 first step**. plumb_bob (5-param) 모자라면 rational (8-param) 또는 fisheye model 검토.

---

## 작업대

- 책상 **가로 55 × 세로 34 cm** (보드 + 로봇 + 작업영역 공유)
- OMX reach (URDF link 합): 일직선 stretched **500mm**, 자세 다양성 확보 영역 ≈ **350-400mm** sphere
- 캘 보드 5×7/25mm ([calibration_workflow.md §5](calibration_workflow.md)) 선정 시 이 작업대 + OMX reach 압박 고려됨
