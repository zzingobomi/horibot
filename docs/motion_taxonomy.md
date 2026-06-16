# Motion Taxonomy

Horibot 의 motion primitive 분류 + 산업 표준 매핑 + 채택 결정.

## 배경

OMX-F 5DOF 시기에 motion API 가 "되는 만큼 만" 짜여 박혔음. 구체적으로:

- `MoveTcp` 라는 이름인데 실제론 trajectory planner 우회 + 절대 target chase. "Move" 컨벤션 위반.
- 5DOF 라 orientation 제어 자체가 안 되니까 quaternion 필드 자체 X.
- joint jog / cartesian jog 같은 산업 펜던트 standard 기능 부재.

SO-101 6DOF 도입 + gamepad mini 펜던트 설계 시작하면서 산업 표준 다시 확인. 결과: 명확한 **3-계층 taxonomy** 가 이미 산업 합의로 있음. 우리는 절반만 구현된 상태였음.

본 문서 = 그 taxonomy 정리 + Horibot 채택 결정 박제. 다음 implement session 시작점.

## 산업 표준 — 3 계층 × 2 입력 공간

motion primitive 는 **3 계층** × **2 입력 공간** = 6 자리 (+ trajectory 확장 변형).

| 계층 | 핵심 차이 | caller 책임 | server 책임 | 안전 패턴 |
|---|---|---|---|---|
| **Trajectory-planned** | 단발 target → planner 가 부드러운 profile 만듦 | 목표 1개 | Ruckig / S-spline 으로 jerk-limited 보간 | trajectory 완료까지 동기 / cancel API |
| **Servo (target chase)** | 외부가 *빠른 rate* 로 절대 target 스트림 → planner 우회 직접 follow | 빠른 rate (UR=500Hz) 절대 target 갱신 | IK + 즉시 publish, stateless | 입력 끊겨도 마지막 target 까지 감 (caller 책임) |
| **Velocity (jog)** | 외부가 속도 벡터 → server 가 그 속도로 motion | 속도 vector + 입력 유지 | 현재 velocity tracking + Ruckig 같은 profile + timeout | **timeout 시 자동 정지** (deadman 자연) |

각 계층 × {joint, cartesian} = 6 primitive. cartesian 의 trajectory 는 변형 4개 (linear / circular / spline / process):

| 계층 | Joint | Cartesian |
|---|---|---|
| Trajectory | `MoveJ` | `MoveL` / `MoveC` / `MoveP` |
| Servo | `ServoJ` | `ServoTcp` (UR `servoc`) |
| Velocity | `SpeedJ` | `SpeedTcp` (linear + angular twist) |

## 브랜드 매핑 (cross-check)

| 우리 | UR (URScript) | ABB (RAPID) | KUKA (KRL) | FANUC |
|---|---|---|---|---|
| `MoveJ` | `movej` | `MoveJ` / `MoveAbsJ` | `PTP` / `SPTP` | `J` |
| `MoveL` | `movel` | `MoveL` | `LIN` / `SLIN` | `L` |
| `MoveC` | `movec` | `MoveC` | `CIRC` / `SCIRC` | `C` |
| `MoveP` | `movep` | (없음) | `SPLINE` | (없음) |
| `ServoJ` | `servoj` | (EGM) | (RSI / FRI) | (Karel stream) |
| `ServoTcp` | `servoc` | (EGM) | (RSI Cartesian) | (Karel stream) |
| `SpeedJ` | `speedj` | (EGM 속도 모드) | RSI velocity | (Karel jog) |
| `SpeedTcp` | `speedl` | (EGM 속도 모드) | RSI Cartesian velocity | (Karel jog) |

ABB 의 EGM (Externally Guided Motion) / KUKA 의 RSI (Robot Sensor Interface) / FRI (Fast Research Interface) 는 servo/velocity 통합 streaming 인터페이스. 의미적으론 같은 자리.

## 펜던트의 jog 패턴 — 산업 표준

펜던트의 cartesian jog 는 **6DOF twist** 입력 = 평행이동 3 + 회전 3:

```
+X / -X    +Rx / -Rx
+Y / -Y    +Ry / -Ry
+Z / -Z    +Rz / -Rz
```

전형적으로 `SpeedTcp` 으로 구현 — 사용자가 "+Rx" 버튼 누르면 `SpeedTcp(linear=(0,0,0), angular=(w,0,0))` publish. **TCP 위치 고정 + 자세만 변경** 케이스 (예: 용접 토치를 끝점 중심 기울이기) 자동 처리.

**ISO 10218 enabling switch** (3-position deadman, 산업 펜던트 안전 필수) → gamepad LT/RT 트리거 hold 로 매핑. 손 떼면 velocity timeout 발동 → 자동 정지. **velocity primitive 가 jog 자리인 핵심 이유**.

## Horibot 채택 결정

### Phase 1 — 한 PR 에 묶음 (다음 implement session)

| 작업 | 상세 |
|---|---|
| **신규** `MOTION_SPEED_TCP` | `SpeedTcpReq {linear: [vx,vy,vz], angular: [wx,wy,wz], frame: "tcp" \| "base"}`. server 가 Ruckig velocity 모드로 추종 + 100ms timeout 정지. |
| **신규** `MOTION_SPEED_J` | `SpeedJReq {velocities: list[float]}` (arm dof 길이). joint velocity 직접 추적 + timeout. |
| **rename** `MOVE_TCP` → `SERVO_TCP` | 의미 정합. `MoveTcpReq` → `ServoTcpReq`. `motion_modes.move_tcp` → `servo_tcp`. frontend `MoveTCP.tsx` UI 라벨 / caller 갱신. `generated/contract.ts` 재생성. |
| **gamepad rewrite** | `SpeedTcp`/`SpeedJ` 사용 mini 펜던트. capability gate (`"gamepad" in capabilities`). target = enabled+capable 1개, N>1 시 fail-fast. mode 토글 (TCP/Joint jog). frame 토글 (TCP/base). deadman (LT hold). B = `CALIB_HANDEYE_CAPTURE`. |
| **host config 등록** | `host_pc.yaml` / `host_dev.yaml` / `host_mock.yaml` 에 gamepad_node 추가. |
| **robots.yaml** | `so101_6dof_0.capabilities` 에 `"gamepad"` 추가. OMX 는 안 줌 (5DOF 펜던트 부적합). |

### Phase 2 — 실제 써본 후 평가

Phase 1 의 6 primitives + gamepad pendant 로 SO-101 캘 / 일반 운용 진행. 다음 검토 대상:

- `MoveL/C/P` 의 orientation 확장 — random palletizing / 비스듬한 면 접근 같은 task driver 가 등장하면.
- gamepad button 매핑 fine-tuning — 실 사용 ergonomics 피드백.

### Phase 3 — 진짜 driver 등장 시 추가

- `ServoJ` — RL / imitation learning replay / 외부 trajectory player 등 외부 컨트롤러가 joint 절대 target 을 빠른 rate 로 보내고 싶을 때.
- 그 외 산업표준 primitive 들이 우리 use case 와 매칭될 때.

**보류 원칙**: driver 없는 primitive 미리 만들지 않음. driver 의 요구를 알고 디자인하는 게 더 정확.

## 각 primitive — semantic 한 줄

이름이 흔들리지 않게 의미 박제. rename / 신규 결정의 anchor.

| primitive | semantic 한 줄 |
|---|---|
| `MoveJ` | Caller 제공 절대 joint target 으로 trajectory-planned (Ruckig) 이동. |
| `MoveL` | Caller 제공 절대 TCP position 으로 cartesian 직선 trajectory-planned 이동. |
| `MoveC` | Caller 제공 via + end TCP position 으로 cartesian 원호 trajectory-planned 이동. |
| `MoveP` | Caller 제공 waypoint list 로 cartesian spline trajectory-planned 이동. |
| `ServoTcp` (← `MoveTcp` rename) | Caller 가 제공하는 절대 TCP target 을 trajectory planner 우회로 즉시 IK + direct publish. caller 가 빠른 rate (50Hz+) 로 갱신 시 chase 패턴. |
| `SpeedTcp` (신규) | Caller 가 제공하는 TCP twist (linear 3 + angular 3) 를 server 가 timeout 까지 추종. 입력 끊김 = 자동 정지. |
| `SpeedJ` (신규) | Caller 가 제공하는 joint velocity 벡터를 server 가 timeout 까지 추종. 입력 끊김 = 자동 정지. |
| `ServoJ` (보류) | 외부가 절대 joint target 을 빠른 rate 로 스트림 — Phase 3 driver 등장 시. |

## gamepad mini 펜던트 — primitive 매핑

```
TCP jog 모드:
  왼스틱 X/Y → SpeedTcp(linear=(vx,vy,0), angular=(0,0,0))
  LT/RT     → SpeedTcp(linear=(0,0,vz), ...)
  오른스틱   → SpeedTcp(angular=(wx,wy,0), linear=(0,0,0))
  LB/RB     → SpeedTcp(angular=(0,0,wz), ...)

Joint jog 모드:
  스틱/트리거 6 축 → SpeedJ(velocities=[v1,v2,...,v6])
  6DOF 면 정확히 일대일 매핑

공통 버튼:
  X = 토크 토글 (기존)
  Y = 홈 (MoveJ to home)
  A = 그리퍼 토글
  B = CALIB_HANDEYE_CAPTURE (캘 모드일 때)
  Back/Start = mode 토글 (TCP / Joint jog)
  LT hold = deadman (안 누르면 모든 motion 토픽 정지)
```

세부 매핑은 implement session 시 ergonomics 보면서 조정.

## References

- [UR ROS2 Driver Position/Velocity Control](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/usage/position_velocity_control.html)
- [UR Forum — Gamepad with SpeedJ or ServoJ](https://forum.universal-robots.com/t/control-robot-from-gamepad-with-speedj-or-servoj/7293)
- [URScript Manual SW5.11](https://s3-eu-west-1.amazonaws.com/ur-support-site/115824/scriptManual_SW5.11.pdf)
- [ABB RAPID Motion Instructions](https://library.e.abb.com/public/688894b98123f87bc1257cc50044e809/Technical%20reference%20manual_RAPID_3HAC16581-1_revJ_en.pdf)
- [KUKA Motion Types Discussion](https://www.robot-forum.com/robotforum/thread/36117-understanding-the-difference-between-motion-types/)
- [Fanuc Frames Convention](https://industrialrobotics.miraheze.org/wiki/Frames)
- [MoveIt Gamepad Teleoperation](https://moveit.picknik.ai/main/doc/how_to_guides/controller_teleoperation/controller_teleoperation.html)
- [ISO 10218-1 Enabling Switch](https://blog.ansi.org/ansi/iso-10218-1-2025-robots-and-robotic-devices-safety/)
