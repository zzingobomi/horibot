# Motion Taxonomy

Horibot 의 motion primitive 분류 + 산업 표준 매핑 + 채택 결정.

## 4 계층 (2026-06-17 정정)

```
Move*    one-shot target motion (trajectory-planned)
Servo*   external absolute target stream — RL / Vision servo
Jog*     human/manual velocity stream — frontend / gamepad
Task*    scripted execution — task_node
```

명명 자리 컨벤션 — *이름 = primitive contract*. 이름 자리 깨면 안 됨.

| 계층 | Joint | Cartesian | caller | server | 산업 표준 매핑 |
|---|---|---|---|---|---|
| **Move** | `MoveJ` | `MoveL` / `MoveC` / `MoveP` | 단발 절대 target | Ruckig jerk-limited 보간 | UR `movej/movel/movec/movep` / ABB `MoveJ/L/C` |
| **Servo** (target chase) | `ServoJ` | `ServoTcp` | *외부 controller* 가 자기가 계산한 *절대 target* 빠른 rate 스트림 | direct IK + publish (planner 우회) | UR `servoj/servoc`, ABB EGM, KUKA RSI |
| **Jog** (human velocity) | `JogJ` | `JogTcp` (linear + angular twist + frame) | *human / gamepad* 가 velocity 보냄 | backend latched ref + 실 dt 적분 → IK + publish | LeRobot delta-pose, UR teach pendant jog |
| **Task** | (n/a — task_node primitive) | | scripted recipe | step DSL 통한 직접 motion 호출 | — |

## 산업 표준 명명 자리 vs 우리 구현

- **Servo (target chase)**: 산업 표준 자리 = 외부 controller (RL / Vision / EGM / RSI / Karel stream) 가 자기가 계산한 *절대 target* 자리 빠른 rate (50Hz+) 으로 스트림. server 자리 = direct IK + publish, planner 우회. UR `servoj` (joint) / `servoc` (cartesian) 가 reference. 우리 caller 자리 = 미래 RL / vision servo policy node.

- **Jog (human velocity)**: 산업 표준 자리 = teach pendant 의 human 조작 jog. caller 자리 = *human / pendant / joystick* 가 velocity 보냄. server 자리 = jerk-limited velocity stream 또는 backend latch + 적분 (LeRobot 방식). 우리 자리 = backend latch + 실 dt 적분 → IK + publish (LeRobot SO100FollowerEndEffector + InverseKinematicsEEToJoints 패턴 자리).

| 우리 | UR (URScript) | ABB (RAPID) | KUKA (KRL) | FANUC |
|---|---|---|---|---|
| `MoveJ` | `movej` | `MoveJ` / `MoveAbsJ` | `PTP` / `SPTP` | `J` |
| `MoveL` | `movel` | `MoveL` | `LIN` / `SLIN` | `L` |
| `MoveC` | `movec` | `MoveC` | `CIRC` / `SCIRC` | `C` |
| `MoveP` | `movep` | (없음) | `SPLINE` | (없음) |
| `ServoJ` | `servoj` | (EGM) | (RSI / FRI) | (Karel stream) |
| `ServoTcp` | `servoc` | (EGM) | (RSI Cartesian) | (Karel stream) |
| `JogJ` | `speedj` (variant) | (EGM) | RSI velocity | (Karel jog) |
| `JogTcp` | `speedl` (variant) | (EGM) | RSI Cartesian velocity | (Karel jog) |

ABB EGM / KUKA RSI / FRI 자리 servo/velocity 통합 streaming. 의미적으론 같은 자리.

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

### Phase 1 — 구현 완료 (2026-06-16)

| 작업 | 상세 | 상태 |
|---|---|---|
| **신규** `MOTION_SPEED_TCP` (backend) | `SpeedTcpReq {linear, angular, frame: "base" \| "tcp"}`. Ruckig `ControlInterface.Velocity` + 100ms timeout + idle grace 0.5s 자동 정지. server-side `PybulletKinematics.tcp_twist_to_joint_vel` 가 Jacobian pseudo-inverse (frame 변환 포함). dof<6 자리 linear-only fallback. | ✅ |
| **신규** `MOTION_SPEED_J` (backend) | `SpeedJReq {velocities}` (arm dof). joint velocity 직접 추적. dof mismatch 시 ValueError. | ✅ |
| **rename** `MOVE_TCP` → `SERVO_TCP` (backend) | `ServoTcpReq {position, quaternion?}` — 6DOF orientation 옵션 (None = position-only). `MotionCommand` 패턴 정식 편입 (기존 dict-API `_srv_move_tcp` 폐기). | ✅ |
| **gamepad rewrite** | capability gate (enabled + `"gamepad"` 정확히 1개, N>1 fail-fast). LT deadman + Back mode 토글 + Start frame 토글. TCP mode 6DOF twist, Joint mode 6축 매핑. 모든 버튼 (X/Y/A/B). | ✅ |
| **frontend 모션패널 — Trajectory 4 탭** | `MoveJ.tsx` / `MoveL.tsx` / `MoveC.tsx` / `MoveP.tsx`. 단발 절대 target. 탭 mount 시 자동 sync (탭 reset → 0 명령 위험 차단), 소수점 `toFixed(2)` 통일. | ✅ |
| **frontend 모션패널 — Velocity 2 탭 (Jog)** | `SpeedJ.tsx` (joint별 −/+ 버튼 hold = 50Hz `MOTION_SPEED_J` publish) / `SpeedTcp.tsx` (6DOF twist 버튼 hold + base/tcp frame 토글). 손 떼면 backend 100ms deadman timeout 자동 정지 — 게임패드 LT hold 와 동일 메커니즘. **2026-06-16 추가 (`MOTION_SPEED_TCP`/`MOTION_SPEED_J` backend Phase 1 시점에 빠져 있던 자리)**. | ✅ |
| **host config 등록** | `host_pc.yaml` / `host_dev.yaml` 에 gamepad 추가. host_mock 은 의도적 제외 (frontend UX 자리, 실 펜던트 의미 X). robots 는 enabled robot 과 일치 (`so101_6dof_0`). | ✅ |
| **robots.yaml** | `so101_6dof_0.capabilities` 에 `"gamepad"` 추가. OMX 안 줌 (5DOF 펜던트 부적합). | ✅ |

**Servo 계층 (ServoTcp / ServoJ) 은 frontend 모션패널에서 빠짐.** ServoJ 는 Phase 3 보류 (driver 없음). ServoTcp 는 정의상 *외부 컨트롤러 50Hz+ chase* 가정이라 단발 패널 자리 의미 미스매치 — 한때 만들어졌던 `frontend/src/components/panels/motion/ServoTCP.tsx` 는 2026-06-16 제거. 운영 caller = 게임패드 (`GamepadNode`) + 미래 RL/외부 driver 만.

### Phase 1.5 (2026-06-17) — Servo / Jog 분리 + SpeedJ/SpeedTcp 폐기

**SpeedTcp/SpeedJ 의 directional transient drift** ([jog_drift_tuning.md](jog_drift_tuning.md)) 진단 결과 root cause 가 *resolved-rate velocity-streaming* 아키텍처 자체 (`pinv(J) @ ramping_twist` 자리 ratio mismatch). LeRobot 본체 + XLeRobot + box2ai-robotics 가 *전부 position-increment (delta-pose 적분)* 으로 짠 자리. 우리 jog 도 그 패턴 채택.

**명명 자리 정정** — 처음 자리 ServoJ/ServoTcp 자리 *velocity input* 자리 박았던 자리 = *Servo 의미 자리 깨는 자리* 자리 (Servo 자리 산업 표준 의미 = caller 가 절대 target 자리). 정석 자리는 **Jog 라는 별도 계층 신설** + **Servo 의미 자리 보존**.

| 작업 | 상세 | 상태 |
|---|---|---|
| **신규 계층** `Jog*` | `JogJReq {velocities}` / `JogTcpReq {linear, angular, frame}`. backend `JogJCommand` / `JogTcpCommand` 가 자기 process joint_cache → ref latch + 실 dt 적분 (SE(3) 적분 자리 scipy `Rotation`) → IK → publish_cmd. service (`MOTION_JOG_J/TCP` 단발) + topic stream (`MOTION_JOG_J/TCP_STREAM` 50Hz). | ✅ |
| **`ServoJ` / `ServoTcp` 자리** | 산업 표준 의미 자리 (절대 target chase, UR `servoj/servoc`) 자리 *보존*. caller 자리 = 미래 RL / Vision servo policy node. service 자리만 노출 (단발). | ✅ |
| **`SpeedJ` / `SpeedTcp` 자리 폐기** | Ruckig velocity stream + `_velocity_loop` + `Synchronization.Phase` fix 자리 + cartesian Ruckig 자리 dead code 자리 전체 제거. `_tcp_twist_to_joint_vel` callback 자리도 제거. | ✅ |
| **gamepad_node** | `MOTION_SPEED_TCP` / `MOTION_SPEED_J` service call → `MOTION_JOG_TCP_STREAM` / `MOTION_JOG_J_STREAM` topic publish. 50Hz service RTT 회피. | ✅ |
| **Frontend** | `ServoJ.tsx` / `ServoTcp.tsx` (Three.js Quaternion 적분 자리) + `SpeedJ.tsx` / `SpeedTcp.tsx` 자리 4 자리 모두 폐기 → `JogJ.tsx` / `JogTcp.tsx` 2 자리. velocity / twist 만 publish — SE(3) 적분 자리 backend SSOT. | ✅ |
| **`motion_node` refactor** | `_dispatch_cartesian` 공통 chain 추출 → MoveL/MoveC/MoveP/ServoTcp service 자리 같은 chain. JogTcp 자리 자체 SE(3) 적분 자리 + tool_offset 매 cycle 자리. | ✅ |
| **mock backend boot fix** | `main.py` 의 instance start 순서 reorder — application_nodes 먼저, device_nodes 다음. 기존엔 motion_node 가 storage retry deadlock 으로 mock backend 90s timeout. | ✅ |

**왜 부드러운가** (jog_drift_tuning.md 참조):
- 매 publish 가 *기하학적으로 정확한 절대 target* → ramp 중 옆으로 새는 자리 0.
- 50Hz × ~2mm/cycle = effective 100 mm/s 인데 cycle 당 step 이 작아 모터 trapezoidal profile 이 *cycle 안에 도달 못 함* → continuous chase → 부드러움.
- backend fresh latch (joint_cache → fk + tool_offset) — 인코더 - ref 누적 drift 차단.

**SSOT 자리**: SE(3) 적분 자리 backend 1자리 (scipy `Rotation`). frontend Three.js / Python gamepad 자리 중복 없음. cross-process safe — frontend 가 모르는 `joint_offset` 자리 backend SSOT.

### Phase 1 구현 중 발견된 trauma (mock 검증으로 잡힌 자리, hardware 안 가도 됐음)

`backend/tests/test_motion_{speed,e2e,gamepad}.py` 37 test 가 잡은 자리:

1. **`TrajectoryRunner._launch` 가 velocity state wipe** — `set_speed_joint` 직후 `_ensure_velocity_streamer` 의 `_launch` 가 `self.stop()` 호출 → `_vel_last_set` 0 reset → streamer 첫 step 에서 timeout 즉시 종료. `_stop_thread` / `_reset_velocity_state` 분리로 fix.
2. **`JointStateCache.subscribe` default robot mismatch** — `subscribe(self)` 가 default robot 토픽 구독. motion_node 가 다른 robot 으로 떠도 default 만 봄. `robot_id=self.robot_id` 명시.
3. **`MotionModes` / `_kinematics` default = so101** — motion_node 가 다른 robot 자리에서 dof / Jacobian mismatch. `MotionModes(robot_id=...)` + `RobotRegistry().get_kinematics(robot_id)`.
4. **PyBullet dof (arm + gripper) vs streamer (arm-only) length mismatch** — `calculateJacobian` 호출 시 silent crash → SpeedTcp publish 0. motion_node 에서 zero-pad / slice.
5. **host_mock 의 robots ↔ robots.yaml enabled mismatch** — host config 의 robot 이 disabled 면 calibration 이 그 robot 의 kinematics initialize 안 함 → motion_node 의 fk 호출 시 RuntimeError. host_mock 의 robots 를 enabled robot 과 일치.

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
| `ServoTcp` | *외부 controller* (RL / Vision servo) 가 자기가 계산한 *절대 TCP target* 을 빠른 rate (50Hz+) 로 스트림 → server direct IK + publish (planner 우회). UR `servoc` / EGM / RSI Cartesian 자리 정석. caller=human jog 자리 X (그건 `JogTcp`). |
| `ServoJ` | *외부 controller* (RL replay / motion capture remap) 가 자기가 계산한 *절대 joint target* 을 빠른 rate 로 스트림 → server direct publish (IK 불요). UR `servoj` / KUKA RSI joint 자리 정석. |
| `JogTcp` | *Human / gamepad* 가 *velocity twist* (linear+angular+frame) 보냄 → backend 가 자기 process joint_cache → fk + tool_offset 으로 *실 끝점 pose* fresh latch + 실 dt SE(3) 적분 → IK → publish_cmd. LeRobot delta-pose 패턴, scipy `Rotation` SSOT. |
| `JogJ` | *Human / gamepad* 가 *joint velocity* 만 보냄 → backend 가 joint_cache (joint_offset 적용 URDF rad) 에서 ref latch + 실 dt 적분 → publish_cmd. IK 불요. cross-process safe (joint_offset SSOT = backend). |

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
