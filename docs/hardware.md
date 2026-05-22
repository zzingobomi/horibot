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
