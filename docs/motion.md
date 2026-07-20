# Motion (통합)

> **통합본 (2026-07-11 문서 다이어트)** — 아래 문서들을 원문 그대로 병합. 옛 파일명으로의
> 링크는 본 문서 내 해당 부(또는 git history). 각 부의 제목/상태 배너는 병합 당시 그대로.
> - `motion.md`
> - `motion.md`
> - `motion.md`


---
---

<!-- ═══════════ [통합 원문] motion.md ═══════════ -->

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

**SpeedTcp/SpeedJ 의 directional transient drift** ([motion.md](motion.md)) 진단 결과 root cause 가 *resolved-rate velocity-streaming* 아키텍처 자체 (`pinv(J) @ ramping_twist` 자리 ratio mismatch). LeRobot 본체 + XLeRobot + box2ai-robotics 가 *전부 position-increment (delta-pose 적분)* 으로 짠 자리. 우리 jog 도 그 패턴 채택.

**명명 자리 정정** — 처음 자리 ServoJ/ServoTcp 자리 *velocity input* 자리 박았던 자리 = *Servo 의미 자리 깨는 자리* 자리 (Servo 자리 산업 표준 의미 = caller 가 절대 target 자리). 정석 자리는 **Jog 라는 별도 계층 신설** + **Servo 의미 자리 보존**.

| 작업 | 상세 | 상태 |
|---|---|---|
| **신규 계층** `Jog*` | `JogJReq {velocities}` / `JogTcpReq {linear, angular, frame}`. backend `JogJCommand` / `JogTcpCommand` 가 자기 process joint_cache → ref latch + 실 dt 적분 (SE(3) 적분 자리 scipy `Rotation`) → IK → publish_cmd. service (`MOTION_JOG_J/TCP` 단발) + topic stream (`MOTION_JOG_J/TCP_STREAM` 50Hz). | ✅ |
| **`ServoJ` / `ServoTcp` 자리** | 산업 표준 의미 자리 (절대 target chase, UR `servoj/servoc`) 자리 *보존*. caller 자리 = 미래 RL / Vision servo policy node. service 자리만 노출 (단발). | ✅ |
| **`SpeedJ` / `SpeedTcp` 자리 폐기** | Ruckig velocity stream + `_velocity_loop` + `Synchronization.Phase` fix 자리 + cartesian Ruckig 자리 dead code 자리 전체 제거. `_tcp_twist_to_joint_vel` callback 자리도 제거. | ✅ |
| **gamepad_node** | `MOTION_SPEED_TCP` / `MOTION_SPEED_J` service call → `MOTION_JOG_TCP_STREAM` / `MOTION_JOG_J_STREAM` topic publish. 50Hz service RTT 회피. | ✅ |
| **Frontend** | `ServoJ.tsx` / `ServoTcp.tsx` (Three.js Quaternion 적분 자리) + `SpeedJ.tsx` / `SpeedTcp.tsx` 자리 4 자리 모두 폐기 → `JogJ.tsx` / `JogTcp.tsx` 2 자리. velocity / twist 만 publish — SE(3) 적분 자리 backend SSOT. | ✅ |
| **`motion_node` refactor** | `_dispatch_cartesian` 공통 chain 추출 → MoveL/MoveC/MoveP/ServoTcp service 자리 같은 chain. JogTcp 자리 자체 SE(3) 적분. | ✅ |
| **mock backend boot fix** | `main.py` 의 instance start 순서 reorder — application_nodes 먼저, device_nodes 다음. 기존엔 motion_node 가 storage retry deadlock 으로 mock backend 90s timeout. | ✅ |

**왜 부드러운가** (motion.md 참조):
- 매 publish 가 *기하학적으로 정확한 절대 target* → ramp 중 옆으로 새는 자리 0.
- 50Hz × ~2mm/cycle = effective 100 mm/s 인데 cycle 당 step 이 작아 모터 trapezoidal profile 이 *cycle 안에 도달 못 함* → continuous chase → 부드러움.
- backend fresh latch (joint_cache → fk) — 인코더 - ref 누적 drift 차단.

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
| `JogTcp` | *Human / gamepad* 가 *velocity twist* (linear+angular+frame) 보냄 → backend 가 자기 process joint_cache → fk 로 URDF EE pose fresh latch + 실 dt SE(3) 적분 → IK → publish_cmd. LeRobot delta-pose 패턴, scipy `Rotation` SSOT. |
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


---
---

<!-- ═══════════ [통합 원문] motion.md ═══════════ -->

# Cartesian Jog Transient Drift — 진단 + Fix 박제

SO-101 6DOF (Feetech STS3215/STS3250) 의 SpeedTcp/SpeedJ jog 자리에서 발생한 **transient drift** (계속 누름 시 *처음 한 번 Z 처지고 그 후 cruise에서 정상 +X*, 찔끔찔끔 자리에 더 심함, wrist down 자세에서 증폭) 진단 + fix 박제. 2026-06-17 session 결과.

**현재 상태 (2026-06-17 update v2)**: **근본 fix 도달 + 4 계층 taxonomy 정리**. root cause 자리 *resolved-rate velocity-streaming* 아키텍처 자체 (`pinv(J) @ ramping_twist`) 였음. **SpeedJ/SpeedTcp 자리 폐기** + 신규 **JogJ/JogTcp** (LeRobot delta-pose 패턴, backend SE(3) 적분 SSOT) + Servo 의미 자리 보존 (절대 target chase = 미래 RL/Vision). frontend / gamepad 자리 모두 같은 wire (`MOTION_JOG_J_STREAM` / `MOTION_JOG_TCP_STREAM`) — directional transient 자리 자체 사라짐 (수학적 보장).

## 신규 ServoJ / ServoTcp = jog의 진짜 fix (2026-06-17)

리서치 (LeRobot SO100FollowerEndEffector, XLeRobot keyboard EE, box2ai-robotics FK/IK) 결과 — **SO-101 cartesian EE 제어 자체는 흔함**, 다만 모두 *position-increment* (delta-pose 적분) 패턴이지 *velocity-streaming/resolved-rate* 아님.

| 자리 | 기존 SpeedTcp | 신규 ServoTcp |
|---|---|---|
| 입력 | twist (linear+angular) | 절대 pose (position + quaternion) |
| 적분 | server `_velocity_loop` 의 cartesian Ruckig | **frontend** ref pose latch + 50Hz dt 적분 |
| Server | pinv(J) @ smoothed_twist → joint vel → Ruckig velocity → publish | IK(target, seed=current_joints) → publish |
| Ramp 자리 | cart Ruckig + joint Ruckig 2 layer ramp | 모터 trapezoidal profile 1 layer (`motors.yaml::profile`) |
| Direction transient | ramp 중 *ratio mismatch* → 옆으로 새는 cartesian drift | **0** — 매 cycle target 이 기하학적으로 정확 |
| Cycle 당 변위 | 가변 (twist × ramp factor) | **~2mm/cycle @ 100mm/s** (cap by motion.yaml::max_trans_vel) |
| Wire | service `MOTION_SPEED_TCP` (RTT 자리) | topic `MOTION_SERVO_TCP_STREAM` (fire-and-forget, LeRobot 표준) |

**왜 잘 동작하나**:
1. 매 publish 가 *절대 target* — ramp 중 옆 방향으로 샐 자리 자체 X (캘이 정확한 한 IK 가 그 pose 풀기만).
2. 50Hz × 1-2mm/cycle = effective 50-100mm/s 인데 cycle 당 *step* 자체가 작아 모터 trapezoidal profile 이 *cycle 안에 도달 못 함* → 매 cycle 모터가 chase 중 → continuous motion.
3. caller (frontend) 가 *fresh latch* — button hold 시작 시 인코더 reading 으로 ref 새로 잡음 → 인코더 - ref 누적 drift 차단.

**잔존 자리** = Feetech PID 응답 자체 (모터 layer). gravity dynamics 의 transient torque lag 가 위치 어긋남 → 모터 PID 가 따라잡음. 단 *방향성 (cartesian Z drift)* 은 사라짐 — 균일한 위치 lag 이라 사용자가 "받아들일만한 자리".

## 박제 폐기 자리 (2026-06-17)

이전 박제 결론 두 개 폐기:

1. ~~"DIY 6DOF cartesian jog는 흔하지 않은 자리"~~ — **틀림**. LeRobot 본체 `SO100FollowerEndEffector`, XLeRobot 키보드 EE, box2ai-robotics/lerobot-kinematics 모두 동일 자리. 차이는 *jog UI 직접 구현* 뿐.
2. ~~"잔존 transient = *물리 layer (Feetech PID + gravity dynamics)* 자리 운명"~~ — **과함**. 대부분은 *velocity-streaming 선택의 산물*. position-increment 로 가니 directional transient 0. 진짜 물리 자리 (PID + gravity) 잔존만 남음.

이 결론 폐기의 결정타 — *왜 +X 누르면 항상 Z 먼저 처짐* 이 깨끗한 방향성을 가졌나는 random backlash 가 아니라 *아키텍처의 결정론적 증상* 이었음.

## 배경 — 증상

사용자 frontend Jog TCP 패널에서 +X 1초 누름:
1. **transient phase (~0.3s)**: EE가 *Z 방향으로 약 1-2cm 처짐*
2. **cruise phase**: 정상 +X 추종

새 단서:
- *찔끔찔끔 burst 자리*에 더 심함 — 매 burst가 *transient에서 끝나고 cruise 못 도달* → drift 누적
- *wrist down 자세*일수록 심함 — shoulder J2 토크 부담 큼
- *위쪽 자세에서 +X = Z 처짐 → 앞으로 / -X = Z 올라감 → 뒤로* — gravity dynamics 시그니처

## 진단 결과 — backend layer 깨끗

### 1. Jacobian frame 검증

[pybullet_kinematics.py:tcp_twist_to_joint_vel](../backend/modules/kinematics/adapters/pybullet_kinematics.py#L234) 의 `pinv(J) @ twist` 결과 *V_recovered = J @ qdot* 출력. 모든 cycle `err_rel = 0.000000` — Jacobian self-consistent, frame mismatch 아님. PyBullet의 `calculateJacobian`이 *world frame Jacobian* 반환 (= URDF base link frame, useFixedBase identity).

### 2. Ruckig state persistent 확인

[trajectory_runner.py:_velocity_loop](../backend/modules/kinematics/trajectory_runner.py#L515) — Ruckig `otg/inp/out` 가 *while loop 밖에서 1회 init* + 안에서 `otg.update(inp, out)` 만 호출. `inp.current_position = out.new_position` 자동 갱신. **trajectory restart 안 일어남**, 산업 pendant 패턴.

### 3. Joint Ruckig ratio mismatch — **root cause**

진단 log 분석 (Synchronization.Phase 적용 전):

```
Cycle 00: target=[0, 0.3035, -0.1569, -0.1466]  out_v=[0, 0.002, -0.002, -0.002]
Cycle 03: target=[0, 0.3037, -0.1581, -0.1456]  out_v=[0, 0.032, -0.0285, -0.0271]
                                                         ↑ J2 11% / J3 18% / J4 18% — ★ ratio mismatch!
Cycle 17: target=[0, 0.3067, -0.1697, -0.1370]  out_v=[0, 0.3067, -0.1692, -0.1376]
                                                         ↑ cruise reached, ratio OK
```

**원인**: Ruckig default = `Synchronization.No` → 각 joint *independent jerk-limited ramp*. *큰 target_velocity (J2=0.30)*가 *작은 (J3=0.16)*보다 ramp 늦음 → out_v ratio ≠ target ratio → cartesian direction transient 동안 깨짐 → visual Z drift.

## 적용된 Fix — 시간순

| # | 변경 | 효과 |
|---|---|---|
| 1 | [motion.yaml](../robot/so101_6dof/motion.yaml) joint jerk/acc 통일 → 일관 ramp | 진단용 (joint별 jerk 불균형 배제) |
| 2 | [trajectory_runner.py:574](../backend/modules/kinematics/trajectory_runner.py#L574) Jacobian @ encoder reading (closed-loop) | belief drift 차단 |
| 3 | [motor_node.py:29](../backend/nodes/device/motor_node.py#L29) `STATE_PUBLISH_HZ = 50` (20 → 50) | encoder reading stale 자리 줄임 |
| 4 | [SpeedTcp.tsx](../frontend/src/components/panels/motion/SpeedTcp.tsx) / [SpeedJ.tsx](../frontend/src/components/panels/motion/SpeedJ.tsx) button up → explicit `target=0` publish | pendant jog semantics. 찔끔찔끔 자리 fix |
| 5 | [trajectory_runner.py:_velocity_loop](../backend/modules/kinematics/trajectory_runner.py#L515) `release_profile/restore_profile` 호출 제거 (SpeedTcp/SpeedJ 자리만) | motors.yaml default profile 유지 → 모터 trapezoidal profile 활성 |
| 6 | [trajectory_runner.py](../backend/modules/kinematics/trajectory_runner.py) Cartesian-space Ruckig (6D linear+angular smoothing) | twist 자체 jerk-limited ramp |
| 7 | ★ **[trajectory_runner.py](../backend/modules/kinematics/trajectory_runner.py) `Synchronization.Phase`** | **root cause fix** — 모든 joint 같은 phase로 ramp → target ratio 매 cycle 100% 유지 → cartesian direction 일관 |

**Synchronization.Phase 시뮬레이션 검증**:

```python
target_velocity = [0.30, -0.16, -0.15]  # target ratio: 1.000 / -0.533 / -0.500
# Synchronization.Phase 적용:
cycle 00: vel=[0.003, -0.0016, -0.0015]  ratio: 1.000 / -0.533 / -0.500 ✓
cycle 05: vel=[0.108, -0.0576, -0.0540]  ratio: 1.000 / -0.533 / -0.500 ✓
cycle 14: vel=[0.300, -0.1600, -0.1500]  ratio: 1.000 / -0.533 / -0.500 ✓
```

모든 cycle에서 *target ratio 100% 유지*. cartesian direction 일관.

## 잔존 자리 — 물리 layer

Phase sync 적용 후 *대부분 자세 cartesian direction 일관*. 잔존:
- *바닥 근처 (wrist down) 자세*에서 여전히 transient
- *위쪽 자세에서 +X 명령 → Z 처짐 → 앞으로* (gravity dynamics)

가설:
1. **자세별 gravity dynamics** — 모터가 *static gravity 보상 torque*에서 *motion torque*로 *전환할 때 순간 lag*. Feetech *position servo*가 gravity feedforward 없음. PID가 *error 누적 → torque 증가 → catch up* 동안 EE 처짐
2. **모터 PID 응답 자체** — STS3250 (shoulder, 50kg·cm) 의 inertia + default PID 응답 시간
3. **부분 sag** — *J4 wrist pitch sag* 또는 *gripper + D405 cantilever 무게*가 자세별 link 변형

## 앞으로 테스트/확인 자리

테스트 그만하는 자리이지만 *나중에 시도 시 참고*용 옵션:

### A. Sag 캘리브레이션 — 부분 fix 추정

- **효과 추정**: *자세별 static gravity 처짐 보상*. transient 시작 자세에서 *moter 명령이 이미 sag 보상* → catch up 자리 줄어듦. 단 *완전 사라짐 보장 X*
- **한계**: 우리 sag 모델 = J2/J3만 ([sag_corrected.py:36](../backend/modules/kinematics/adapters/sag_corrected.py#L36)). *wrist sag (J4 pitch sag, gripper cantilever)*는 안 잡힘
- **수치 측정 가치**: SO-101의 *실 sag 크기 (mm vs cm)*가 *측정됨*. 이게 *jog drift의 *얼마가 sag 기여*인지 판정
- **절차**: [docs/calibration.md](calibration.md) — Hand-Eye 캘과 같은 자세 패턴 + sag 계산

### B. Feetech PID 튜닝 — STS3250 P gain 올리기

- **효과 추정**: 모터 응답 빠르게 → transient 짧아짐
- **위험**: EEPROM write ([feetech_driver.py:set_position_pid](../backend/modules/motor/adapters/feetech_driver.py#L239)). Lock pattern으로 *brick risk 낮음*, 잘못된 값이면 oscillation
- **반복 cycle**: P gain 단계별 올리며 *visual + transient log* 측정
- **현재 default**: motors.yaml에 PID 설정 X = Feetech 공장 default. 보수적
- **시도 순서**: J2 (STS3250 shoulder) P=32 (default) → 48 → 64 → ... overshoot까지 측정

### C. Goal_Position + Goal_Speed sync write

- **효과 추정**: backend가 매 cycle *position + velocity 함께 publish*. 모터 *내장 trapezoidal profile* + *backend Ruckig velocity* 두 layer 동기. *모터가 *backend의 의도 velocity 직접 추종*
- **변경 자리**: [motor_node.py](../backend/nodes/device/motor_node.py) 의 `MOTOR_CMD_JOINT` 메시지 schema 확장 + driver의 `set_goal_positions_sync` + `set_profile_velocities_sync` 매 cycle 호출
- **트레이드오프**: Ruckig output velocity 변환 (rad/s → motor raw step/s) 필요, 큰 코드 변경

### D. Encoder velocity feedback closed-loop

- **효과 추정**: backend가 *모터 실 velocity (encoder finite difference)* 측정 + 명령 velocity 와 비교 + *error를 target_velocity 보정에 누적*. 모터 lag을 backend layer가 *실시간 보상*
- **변경 자리**: [trajectory_runner.py:_velocity_loop](../backend/modules/kinematics/trajectory_runner.py#L515) 매 cycle encoder velocity 계산 + Jacobian @ encoder + velocity error feedback
- **트레이드오프**: 가장 산업 표준 closed-loop control. 큰 코드 변경. 그러나 *hobby motor 한계 어차피 잔존*

### E. 모터 업그레이드 — Dynamixel XM, 산업 grade

- **효과**: 진짜 산업 grade response. transient 완전 사라짐 예상
- **트레이드오프**: $$$ + 다른 robot type 통합 작업

### F. 인정 + 운용 (현재 자리)

- jog는 *대략 이동/teach* 용도로 운용 OK
- 정밀 cartesian motion 필요한 자리는 *MoveL/MoveC/MoveP* (trajectory 패스) — jog의 transient 자리와 **무관**한 코드 경로. trajectory가 *전체 path generate + Ruckig position mode + release_profile* 사용해 자체적으로 부드러움 보장
- backend layer는 산업 표준 도달, study output 충분

## 핵심 결정 박제

- ~~**DIY 6DOF cartesian jog는 흔하지 않은 자리**~~ — **폐기** (위 §박제 폐기 참조). LeRobot 자리 흔함, 우리만 resolved-rate 선택했음.
- **Synchronization.Phase 가 SpeedTcp/SpeedJ 자리의 backend layer 마지막 cap** — root cause 의 *resolved-rate aware fix* 자리. SpeedTcp/SpeedJ 사용 유지 시 의미 있는 박제. ServoTcp/ServoJ 자리는 적분 자체가 server-side 아니라 무관.
- **진짜 root cause fix = position-increment (ServoJ/ServoTcp)** — backend layer 의 마지막 cap 이 아니라 *아키텍처 선택의 마지막 cap*. SpeedJ/SpeedTcp 의 directional transient 는 *수학적으로* 사라짐 (방향이 매 cycle 기하학적으로 정확).
- **잔존 transient = *물리 layer (Feetech PID + gravity dynamics)* 자리** — 그 자리 자체는 그대로 남음. backend 자리에서 sag 캘 / Feetech PID 튜닝 (옵션 A/B) 으로 줄일 수 있지만 *완전 보장 X*. 사용자가 받아들이는 자리.

## References

- [motion.md](motion.md) — 3 계층 motion primitive 분류, Phase 1 채택
- [calibration.md](calibration.md) — 4종 캘리브레이션 절차
- [calibration_apply_flow.md](calibration_apply_flow.md) — sag/link/joint offset 적용 메커니즘
- [Ruckig Synchronization](https://github.com/pantor/ruckig) — Phase/Time/TimeIfNecessary/No 옵션
- [Feetech LeRobot motors](https://www.mintlify.com/huggingface/lerobot/motors/feetech) — STS3215 default 패턴
- [LeRobot configure_motor issue #673](https://github.com/huggingface/lerobot/issues/673) — Maximum_Acceleration 설정


---
---

<!-- ═══════════ [통합 원문] motion.md ═══════════ -->

# Frontend URDF visual ↔ backend corrected TCP mismatch (2026-06-22, 다음 세션 논의 anchor)

> SO-101 + D405 setup 에서 3D viewer 의 *URDF tcp link visual* (빨간 box) 와
> *TCP 좌표축* (label="TCP", backend `MOTION_STATE_TCP`) 가 위치 어긋남. ≤ 4°
> 정도, 시각 cosmetic 만 — robot 명령 / 캘 / motion 다 좌표축 (corrected FK)
> 기준이라 동작 영향 0. 사용자가 "URDF 잘못 맞춘 거 아니냐" 로 인지해 발견.
> 진단 끝, 수정 보류 (추후 논의 후 진행).

## 증상

[Move 페이지 / Calibrate 페이지 공통] 3D viewer 에서:
- URDF 의 `<link name="tcp">` visual (작은 빨간 box, [so101_6dof.urdf:448-454](../robot/so101_6dof/urdf/so101_6dof.urdf#L448-L454)) 가 한 자리
- TCP 좌표축 (X/Y/Z arrow + "TCP" label) 이 그 옆 살짝 떨어진 자리
- 둘이 정확히 겹쳐야 *시각적으로 일관* 하지만 어긋남

## Root cause — 두 FK chain 이 서로 다른 URDF + sag 유무

같은 joint angle (backend `MOTOR_STATE_JOINT`) 을 입력으로 받지만:

```
joint angle ─┬─→ frontend urdf-loader (RobotModel.tsx)
             │     ├─ 입력: /robot/.../so101_6dof.urdf  (정적 마운트 raw 원본)
             │     ├─ sag 보정: 없음
             │     └─ 결과: tcp link 위치 → 빨간 box 렌더
             │
             └─→ backend motion_node
                   ├─ 입력: 같은 URDF 를 in-memory patch (link_offset 적용)
                   │       pybullet_kinematics.py:96 `patch_urdf_text`
                   ├─ sag 보정: SagCorrectedKinematics Decorator (J2/J3 처짐)
                   └─ 결과: MOTION_STATE_TCP publish → 좌표축 렌더
```

차이:
| | URDF 자체 | sag |
|---|---|---|
| 빨간 박스 (frontend) | raw 원본 | ❌ |
| 좌표축 (backend) | link_offset patch | ✅ |

캘 5종 모두 active (`storage/horibot.db::calibration_results WHERE is_active=1` —
joint_offset / link_offset / sag / hand_eye / intrinsic) 라 mismatch 가 visible.

## 왜 frontend 가 patched URDF 를 못 쓰나

- backend [pybullet_kinematics.py:103-122](../backend/modules/kinematics/adapters/pybullet_kinematics.py#L103-L122) 가 patch 결과를 `tempfile` 로 1회성 write → `loadURDF` → `unlink`. 디스크에 안 남김.
- frontend urdf-loader 는 [bridge/zenoh_bridge.py](../backend/bridge/zenoh_bridge.py) 가 `/robot` 으로 정적 마운트한 **raw 파일** HTTP fetch — patch 가 안 박힌 원본.
- sag 는 자세 의존 보정 (Decorator) 이라 *URDF 수정으로 표현 불가능* — frontend FK 에 넣을 방법 자체가 없음 (joint angle 마다 다른 보정량).

## Impact

| | 영향 |
|---|---|
| MoveL / MoveC / MoveP / ServoTcp / JogTcp 도달점 | ✅ 좌표축 기준 (corrected FK) — 빨간 박스 무시 |
| Hand-eye 카메라 frustum / cameraMatrix | ✅ 좌표축 기준 |
| PyBullet IK target | ✅ 좌표축 기준 |
| Live PointCloud transform | ✅ 좌표축 기준 (`cameraMatrix = tcpMatrix · handEye`) |
| 빨간 박스 위치 | ❌ raw URDF FK — 시각 표시에만 사용, 명령 chain 어디에도 안 들어감 |

→ **시각 cosmetic 만**. critical X. ([Container.tsx:72-76](../frontend/src/components/scene/Container.tsx#L72-L76) 에 "critical 아니라 미룸" 으로 박혀있는 자리.)

## Fix 옵션 (보류, 추후 논의)

### 옵션 A: bridge 가 patched URDF 를 endpoint 로 서빙

- bridge 에 `GET /robots/{robot_id}/urdf` 추가 — `RobotRegistry.get_kinematics(id)` 가 들고 있는 patched URDF 텍스트 (또는 path) 반환
- frontend urdf-loader 가 정적 `/robot/<type>/urdf/...` 대신 robot 별 patched endpoint fetch
- 단 sag 는 여전히 안 들어감 → ≤ 4° 잔존 (자세 따라 변동)
- **장점**: 구현 간단. URDF 가 robot instance 의 link_offset 반영
- **단점**: sag mismatch 잔존. urdf-loader 가 robot 별 URDF 받는 lifecycle 변경 필요

### 옵션 B: backend 가 전체 link pose topic publish + frontend urdf-loader 자체 FK off

- backend 가 `MOTION_STATE_LINKS` (또는 비슷한 이름) topic 으로 *각 link 의 corrected world pose* 매 motor update 마다 publish (joint state 기반)
- frontend `RobotModel.tsx` 가 urdf-loader 의 자체 FK 끄고 각 link 를 backend pose 로 직접 setMatrix
- **장점**: 완전 일관. sag 도 반영. *진짜* SSOT 정석 (캘/sag 가 backend 한 곳에만 산다는 본 architecture 원칙과 align)
- **단점**: urdf-loader 의 자체 FK 끄는 API hook 필요 (gkjohnson loader 가 지원하는지 확인). bandwidth 살짝 증가 (link 수 × matrix 4×4). RobotModel 자리 자체 FK chain 다 갈아엎기

### 옵션 C: 그대로 두기 (현재)

- Container.tsx 코멘트에 박혀있듯 "≤ 4°, critical 아님"
- 캘 정확도 / 명령 / 모션 다 backend SSOT, 빨간 박스는 frontend cosmetic
- **단점**: 새 사용자/개발자가 매번 헷갈림 (사용자가 "URDF 잘못 맞춘 거 아니냐" 로 발견). 학습 곡선 비용

## 결정 사항 (추후 논의 항목)

1. 옵션 A / B / C 중 선택
2. 옵션 B 의 경우 — urdf-loader 가 외부 link pose 받는 API 존재 여부 (gkjohnson urdf-loaders 문서 확인 자리)
3. 옵션 A 의 경우 — patched URDF 가 robot instance 별 다르므로 URL scheme 결정 (`/robots/{id}/urdf` vs `/robot/instances/{id}/urdf` 등)
4. 빨간 box 자체 제거 옵션 — `<link name="tcp">` 의 `<visual>` 빼고 backend 좌표축만 시각에 남기는 것. 가장 간단하지만 "URDF 에 grasp point 표시" 의도 사라짐

## 관련 코드 위치

- Backend FK (corrected): [pybullet_kinematics.py](../backend/modules/kinematics/adapters/pybullet_kinematics.py) + [sag_corrected.py](../backend/modules/kinematics/adapters/sag_corrected.py)
- URDF patch: [urdf_patcher.py](../backend/core/coords/urdf_patcher.py) + `patch_urdf_text`
- TCP topic publish: [motion_node.py:580-595](../backend/nodes/device/motion_node.py#L580) (`_publish_tcp_loop`)
- Frontend tcpMatrix 수신: [Container.tsx:77-107](../frontend/src/components/scene/Container.tsx#L77-L107)
- Frontend URDF FK (raw): [RobotModel.tsx](../frontend/src/components/canvas/3d/RobotModel.tsx) (gkjohnson urdf-loader)
- 정적 URDF 마운트: bridge 의 `app.mount("/robot", StaticFiles(directory=...))`

## 관련 문서

- [calibration_apply_flow.md](calibration_apply_flow.md) — 캘 4종이 어디서 적용되는지 (link_offset 의 URDF in-memory patch 가 backend 한정인 사실 자리)
- [multi_robot_architecture.md](multi_robot_architecture.md) §3.1 — Kinematics layer decorator chain
- [move_page_pointcloud_issues.md](move_page_pointcloud_issues.md) #5 — *별개* 이슈 (URDF joint limits clip 으로 인한 URDF FK 자체 오류). 본 이슈는 limit 가 아니라 link_offset+sag 차이라 root cause 다름.


---
---

<!-- ═══════════ MoveL 꺾임 / IK 잔차 / refine 조사 (2026-07-20) ═══════════ -->

# MoveL 직선 꺾임 & IK 위치잔차 — 조사 + 구현 (2026-07-20)

> **다음에 이 이슈를 제대로 팔 때 같은 논의를 처음부터 반복하지 않기 위한 박제.**
> §1-9 = 회사 조사 기록 (그 세션엔 코드 안 고침). **§10.E = 집 구현 완료
> (2026-07-20)** — §5/§10.D 의 주 경로 conditional refine + §7 관측성 로깅을
> backend 에 구현, sim 4계층 통과 + 타이밍 실측 완료. **실물 파지 재시도율
> before/after 만 사용자 HW 세션에 남음.** 프론트 SpeedDots 오버레이는 별개 완료.

## 1. 증상

MoveL 프리뷰(motion_preview)에서 **경로 끝부분에서 TCP 트레이스가 직선에서 꺾인다.**
트레이스는 각 프레임 관절각의 FK 라, 꺾임 = **IK 가 직선 위 목표점을 정확히 못 맞췄다**는
신호. (프리뷰는 motion 과 같은 IK·runner 라 실물에서도 동일하게 벌어짐 — 착시 아님.)

## 2. 근본 원인 — 수치 IK 트레이드오프 + 침묵 tolerance + 주 경로 refine 없음

- **수치(반복) IK 특성**: PyBullet `calculateInverseKinematics` 는 위치+자세를 함께
  최소화 → 자세가 빡센 지점에서 **위치를 내주고 자세를 맞추는** 해로 수렴.
  자세 slerp 가 경로 진행률에 동기라, 끝(frac→1)에서 목표 자세가 100% 완성되며
  그 자세가 손목 한계에 가까우면 위치오차가 커짐 → 꺾임.
- **침묵 tolerance**: `_ik_from_seed` 는 위치오차(`error`)를 **계산은 하지만**,
  `IK_POS_ERROR_LIMIT`(현 **10mm**) 안이면 **로그 없이 그대로 채택**. 즉 직선 위 임의 점이
  최대 1cm 벗어나도 feasible 로 통과 → 정확히 프로젝트가 경계하는 "침묵 fallback/tolerance".
- **주 경로에 refine 없음**: `ik()` 흐름 = ① seeded 단일 IK(refine X) 성공하면
  **즉시 반환**(`if sol is not None: return sol`) → ② 실패 시에만 `_ik_walk`(refine O)
  → ③ random restart. 문제는 **seeded 가 6~10mm 로 "성공" 판정되면 walk 을 안 거치고
  그 꺾인 해를 그대로 씀.** walk 이 3mm 로 다듬을 수 있는데도 short-circuit.

## 3. 왜 analytic IK 는 이런 게 없나 (개념 정리 — 재설명 방지)

- 해석적(closed-form) IK 는 최소화가 아니라 **기구학 방정식을 대수적으로 정확히 푸는**
  것. 결과 = "정확한 관절각" 또는 "해 없음(도달불가)". **tolerance·잔차·refine·꺾임
  개념 자체가 없음.**
- 단 **Pieper 조건** 필요: 연속 3축이 한 점에서 교차(구형 손목)하거나 3축 평행.
  산업용 6축(UR/ABB/KUKA)이 구형 손목으로 설계되는 이유 = 해석적 IK 를 쓰려고.
- **so101 은 5→6DOF 개조**라 이 구조를 만족 안 할 가능성 큼 → **애초에 닫힌 해가 없어
  수치 IK 가 강제됨** (게을러서가 아님). 확인하려면 URDF 마지막 3관절 축/원점이 한 점에서
  교차하는지 검사 (미수행 — 다음에 IKFast 검토 시). 특이점 velocity 폭발은 두 방식 공통.
  > **2026-07-20 정정 (§10):** 이 가설 **틀림.** so101 은 마지막 3축 교차(구형 손목)는
  > 아니지만 **joint 2,3,4 가 평행** → Pieper 의 "3평행축" 조건으로 **닫힌 해 존재**(UR 과
  > 동일 클래스). 위 검사가 교차만 보고 평행 route 를 놓친 것. 꺾임 원인도 기구학이 아니라
  > **주경로 refine 부재**로 실측 확정 — §10.

## 4. 이미 측정된 사실 (repo 내 근거 — 재측정 불필요)

pybullet.py `_WALK_REFINE_ITERS` 주석 (2026-07-16 실측):
> 단일 IK 호출이 자세 맞추며 위치를 **~15mm 트레이드오프** → **결과를 seed 로 1회
> 재호출만으로 5mm/0.05° 수렴.** 상한 5 는 보수 여유.

즉 **refine(결과-seed 재해) 로 ~15mm → ~5mm 회수 가능**이 이미 검증됨. walk 은 이걸
쓰고, 주 경로는 안 씀 — 이 격차가 본 이슈의 핵심.

## 5. 제안 (구현 대상 — 아직 안 함 → **§10.D 에서 "넣기로" 결정, 조건·검증 포함**)

- **조건부 refine 을 주 경로에 추가**: seeded 성공 직후 잔차가 임계값(예: **>3mm**)이면
  결과를 seed 로 재해(1~few 회). 쉬운 해(<3mm)는 **스킵 → 비용 0**. `_ik_from_seed`
  가 이미 `error` 계산하므로 분기만 추가(FK 추가비용 없음). **작업 종류(파지 등)로
  하드코딩하지 말 것** — 트리거는 "잔차", 파지는 자연히 걸림.
- **게이트(`IK_POS_ERROR_LIMIT`)는 10mm 유지.** 조이는 건 별개 · 나중 · 선택.
- **refine 없이 게이트만 조이지 말 것** (5mm 등) — 실측됨: **도달불가 폭증** (단일
  해가 5mm 를 못 넘어서). refine 이 5mm 바닥을 만들어준 *뒤에만* 조일 수 있고, 그때도
  바닥(~5mm) 밑으론 두지 말 것 (예: 6mm).

## 6. 실패율/속도 분석 (걱정 대응)

- **refine 은 IK 실패를 늘리지 않고 줄임**: 지금 실패의 한 축 = 단일 해가 10~15mm 라
  게이트에 걸려 None. refine 이 ~5mm 로 회수 → **통과**. 게이트 유지 시 "통과하던 건
  다 통과 + 실패 일부 회복" = 순이득. 실패가 느는 유일 경우 = 게이트를 refine 바닥
  밑으로 조일 때.
- **속도 미지수 = `resolve_reachable`(파지 후보 스크리닝)**: 이 경로는 **자세 빡센
  후보가 다수** → 그것들이 곧 refine 트리거. "쉬운 건 스킵" 혜택이 여기선 작을 수 있어
  후보 수백 × refine 비용이 붙을 수 있음(IK 예산 상수 `_SCREEN_/_GROUP_/_PATH_IK_BUDGET`
  로 관리되는 자리). **"안 느리다" 단언 금지 — 구현 시 before/after 타이밍 실측 필수.**
- MoveL 실행 경로(50Hz, 매 틱 IK 1회, 연속 seed)는 조건부 refine 거의 공짜.

## 7. PnP 간헐 파지실패와의 연결 — **가설 (미확정)**

증상: 픽앤플레이스 중 **한 번 헛잡고, 재시도 시 집힘.** 이게 본 이슈일 수 있음:
- IK 는 (target, seed) 에 결정적 (rng fresh — 2026-07-09 수정). 그런데 재시도 때 **팔이
  움직여 seed 가 달라짐** → 같은 목표라도 다른 해. 1차 8mm 어긋난 해로 헛잡음 → 2차 다른
  seed 로 2mm 해 → 집힘. **지금 구조(6~10mm 조용히 통과)와 정확히 일치.**

**단 경쟁 원인이 동급/더 큼 — 지금은 못 가림:**
- **기구학 절대정확도 ~1-2cm (자세의존)** — 예전 파지실패 *진짜 근본*으로 결론난 것
  (project_grasp_depth_rootcause). IK 잔차(5-10mm)보다 크고 **refine 으로 안 줄어듦**.
- 인지(검출) 노이즈, 물리 변수(물체 밀림/미끄러짐).

**확인 방법 (추측 말고 데이터)**: 지금 로그엔 채택 해의 잔차가 없음. servo_pick 실패
분석 trace 에 **시도별 IK 잔차 + 실제 도달 위치오차**를 실으면 → "헛잡음이 잔차 큰 것과
상관되나?" 가 로그에서 갈림. 상관 O = 본 이슈 확정(refine 이 답) / 잔차 작은데 실패 =
IK 아님(절대정확도·인지). **이 로그 추가가 제대로 파기 전 첫 수.**

## 8. 하지 말 것 / 재론 금지

- **refine 없이 게이트만 조이기** (§5) — 도달불가 폭증, 실증됨.
- 파지 실패를 **"D405 저텍스처 depth / σ"** 로 재진단 — 이미 오진으로 기각 확정
  (project_grasp_depth_rootcause). 본 이슈는 그것과 무관.
- refine 을 파지 전용 분기로 하드코딩 — 잔차 트리거로 일반화.

## 9. 관련 코드 위치

- IK 흐름 / seeded short-circuit / walk refine: [pybullet.py](../backend/modules/motion/adapters/pybullet.py) `ik` / `_ik_from_seed` / `_ik_walk`
- 상수: `IK_POS_ERROR_LIMIT`(10mm) / `IK_TOLERANCE`(1e-4) / `_WALK_REFINE_ITERS`(5)
- MoveL 실행 루프 + 진단 로그(잔차 없음): [trajectory_runner.py](../backend/modules/motion/trajectory_runner.py) `_cartesian_loop` / `_log_cart_diag`
- 경로 사전검증: [module.py](../backend/modules/motion/module.py) `_linear_path_blocker`
- 프리뷰(같은 IK/runner 재사용): [motion_preview/module.py](../backend/modules/motion_preview/module.py) `plan_trajectory`
- 속도 점 오버레이(구현됨): [MovePreviewPanel/scenePart.tsx](../frontend/src/components/panels/MovePreviewPanel/scenePart.tsx) `SpeedDots`

## 10. 2026-07-20 업데이트 — 회사 UR sim(rnd_motion) 짜다 역으로 실증 + Pieper 정정

회사 프리뷰용 clean sim 모듈 `rnd_motion`(자체 pybullet IK + **결과-seed refine** + Ruckig)을 만들다가 본 이슈의 원인이 **실측으로 확정**됨. **수정+실물 테스트는 집 트랙에서 사용자가 직접 (아래는 근거 박제).**

### A. so101 은 Pieper 만족 — §3 가설 정정
§3 은 "so101 이 구조(구형 손목)를 만족 안 할 가능성 큼 → 닫힌 해 없음"이라 했는데, so101 URDF(base·calibrated 둘 다)를 pybullet 로 실측하니 **joint 2,3,4 축이 평행**(|dot|=1.000 / 0.9997). Pieper 의 **"3연속 평행축"** 조건 충족 = **닫힌 해 존재, UR 과 같은 클래스.** so101 은 (UR 처럼) 구형 손목이 아니라 **평행축**으로 만족한다 — §3 이 "마지막 3축 교차"만 보고 평행 route 를 안 본 것. 즉 **"so101 은 기구학이 나빠 수치 IK 강제" 는 오답.**

### B. 꺾임 = 기구학 아님, 주경로 refine 부재 (실측)
so101 calibrated URDF, MoveL 3cm + 끝 자세회전 스윕. **단발 IK(현 주경로) vs 결과-seed refine** 의 직선이탈 max:

| 끝 자세회전 | 단발 IK | refine |
|---:|---:|---:|
| 0° | 0.4mm | 0.1mm |
| 40° | 6.5mm | 0.1mm |
| 80° | 35.2mm | 0.6mm |
| 120° | 51.9mm | 0.6mm |

→ **도달 가능한 TCP인데** 단발 IK 는 자세 요구가 커질수록 위치를 내주며 잔차 폭증(=꺾임). **결과를 seed 로 재해(refine)하면 52mm→0.6mm 로 붕괴.** §5 의 "주경로 conditional refine" 이 실제 답임이 실물 URDF 로 확정. (rnd_motion 은 이 refine 을 넣어 UR·so101 모두 sub-mm — `--host rnd_so101` 로 브라우저 비교 가능.)

### C. 결론: 이건 실 버그 (집 수정 대상)
주경로가 ≤10mm 를 **refine 없이 조용히 통과**([pybullet.py](../backend/modules/motion/adapters/pybullet.py) `_ik_from_seed`) → 실 로봇 MoveL/servo 가 자세 빡센 구간에서 **cm급으로 어긋남**(침묵 tolerance, PnP 헛잡음 §7 유력). walk/restart 는 이 잔차엔 과공학 — **값싼 주경로 refine 이 정답.** 단 restart 는 cold-start branch 탐색(다른 팔꿈치/손목 config)엔 여전히 용도 있음(refine 은 basin 내 다듬기라 branch 못 바꿈).

**수정 방향(집 트랙):** §5 대로 주경로에 conditional refine(잔차>임계면 결과-seed 재해) + resolve_reachable 타이밍 실측(§6) + 실물 검증. **rnd 작업과 별개.**

**재현법(스크립트는 scratchpad 비영속):** pybullet 로 (1) 각 관절 월드축 뽑아 |dot|(평행)/손목 선-선 거리(교차) 검사, (2) MoveL 자세스윕을 `_solve_once`(단발) vs `ik`(refine)로 직선이탈 비교.

### D. 결정 (2026-07-20) — **집 motion 주경로에 conditional refine 넣는다**

§5 제안을 "넣기로" 확정 (근거 = 위 B 실측 + C). 단 아래 조건·검증 준수. **집 트랙 작업 (rnd 와 별개) — 구현+실물 검증까지 한 단위.**

- **넣는 이유:** ≤10mm 침묵 통과 버그 제거 → MoveL/MoveJ 실행이 자세 빡센 구간에서 최대 1cm 어긋나던 것 sub-mm 로. seed 의존이라 **PnP 간헐 헛잡음(§7)의 한 원인** 제거. 조건부라 평상시 비용 0.
- **조건부로:** seeded 성공 직후 위치잔차 > 임계(예: 3mm)일 때만 결과-seed 재해(1~few 회). 쉬운 해(<임계)는 skip = 비용 0. `_ik_from_seed` 가 이미 error 계산하므로 분기만.
- **반드시 타이밍 실측:** `resolve_reachable`(52가족 스크리닝)에 후보마다 refine 붙으면 느려질 수 있음(§6). before/after 실측 필수 — **"안 느리다" 단언 금지.**
- **게이트(`IK_POS_ERROR_LIMIT` 10mm)는 refine 바닥 밑으로 조이지 말 것** (§5 — refine 없이 조이면 도달불가 폭증, 실증됨).
- **넘치게 기대 금지:** 절대정확도 ~1-2cm(더 큰 파지 실패 원인, project_grasp_depth_rootcause)는 **안 고침** — refine 은 정밀도지 모델오차 교정 아님. 52가족 도달성 탐색·인지 방어(score/width/base_z/robot-base 컷)는 **별개 축** — refine 이 안 없앤다.
- **검증 완료 기준:** 구현 + resolve 타이밍 before/after + **실물 파지 재시도율 before/after** (servo_pick trace 에 시도별 IK 잔차+도달오차 로깅 = §7 첫 수 겸용).

### E. 구현 완료 (2026-07-20 집) — sim 4계층 + 타이밍 실측 통과, 실물만 남음

§5/§10.D 대로 backend 에 구현. **"sim 으로 증명된 것 vs 실물 미지수" 구분(작업 방식 원칙):**

**구현 (sim 증명 완료):**
- **주 경로 conditional refine** — [pybullet.py](../backend/modules/motion/adapters/pybullet.py) `_ik_from_seed`: 단발 IK 후 위치잔차 > `_IK_REFINE_THRESHOLD_M`(3mm)일 때만 결과-seed 재해(`_IK_REFINE_ITERS`=5, 임계 도달 시 조기 종료). **게이트(`IK_POS_ERROR_LIMIT` 10mm) 검사 앞**에서 도므로 게이트에 걸릴 단발 잔차도 회수해 통과 → `ik()` 3경로(seeded/walk 위치-only/restart) 전부 자동 적용. 게이트는 그대로(안 조임 — §8).
- **§7 관측성 로깅** — ① `resolve_reachable`([module.py](../backend/modules/motion/module.py)) 채택 그룹 IK 위치잔차 max 를 info 로그 (기존 타이밍 로그와 쌍) = 파지 헛잡음↔잔차 상관 분석의 첫 수. ② MoveL 진단([trajectory_runner.py](../backend/modules/motion/trajectory_runner.py) `_log_cart_diag`)에 이동 중 채택 IK 해의 위치잔차 max 필드 추가 (runner 에 `fk` 콜백 신설 — 구성 2곳 motion/motion_preview 갱신).
- **회귀 테스트** — [test_kinematics.py](../backend/tests/modules/test_kinematics.py) `test_conditional_refine_recovers_pose_hard_residual`(tilt 사다리 스윕 집계: 게이트 초과 후보 회수 + 최대/평균 잔차 감소, refine 제거 시 refined==raw 로 세 단언 동시 붕괴) + `test_refine_easy_solution_stays_within_limit`(쉬운 해 비용 0 경로 무결성).
- **4계층 검증**: ruff/pyright clean · fast loop 373 passed · sim motion 38 passed (test_motion/preview/calibration/kinematics).

**타이밍 실측 (§6 "안 느리다 단언 금지" → 데이터):** so101 URDF, nominal 파지 자세 + 손목 tilt 사다리(도달가능 63후보, seed=nominal). 단발 `_ik_raw` vs 주 경로 `_ik_from_seed`:

| | per-call median | 잔차 mean | 잔차 max |
|---|---:|---:|---:|
| 단발(refine 전) | 0.55ms | 7.03mm | **17.62mm** (게이트 초과=기각) |
| 주 경로(refine) | 1.38ms | 2.45mm | 6.62mm (전부 통과) |

→ refine 이 **게이트에 걸릴 단발(17.6mm)을 6.6mm 로 회수** = §6 "refine 은 실패를 늘리지 않고 줄인다" 실증. 비용은 후보당 median +0.8ms — 52가족×수pose ≈ +0.5s 로 resolve 예산(~8s, 실패 그룹 restart 가 지배) 안. (재현 스크립트는 scratchpad 비영속.)

**실물에만 남은 미지수 (sim 이 못 주는 것):**
- refine 후 실 MoveL/servo 의 직선 추종·파지 정확도가 실제로 개선되나 — TCP 잔차의 실물 매핑.
- **§7 가설 판정**: servo_pick trace 의 채택 잔차 로그로 "파지 헛잡음이 IK 잔차와 상관되나" → 상관 O = refine 이 재시도율 낮춤(본 이슈) / 잔차 작은데 실패 = IK 아님(절대정확도 ~1-2cm·인지, refine 무관 §8). **실물 파지 재시도율 before/after 가 최종 판정** — 아직 미측정.
- resolve 타이밍은 sim(DIRECT) 실측치라 실물 배포(Pi)에서 절대값 다를 수 있음 — 상대 오버헤드는 유지 예상.

**§10.E 후기 (2026-07-20 밤, 실물 첫 런):** refine 첫 구현이 **발산 미방어**로 실물 52가족 전멸 유발 — 결과-seed 재해가 나빠질 수 있는데(7mm→12mm) 마지막 반복을 그대로 반환해 "단발이면 통과(≤10mm)했을 후보"가 게이트 초과 None 으로 뒤집힘 + restart 200×refine 5 로 27s. **fix = best(최소잔차) 추적**(refine 은 단발보다 절대 안 나빠짐 — §6 불변식 코드 보장) + **restart probe 는 refine 제외, 승자만 1회 refine**. 회귀 테스트에 "단발 통과 후보를 refine 이 뒤집으면 실패" 불변식 잠금.

### F. 2026-07-20 전멸 감사 — false negative 0, 진짜 도달불가 51/52 (근본 방향 전환점)

그날 밤 실물 전멸 런(검출 0019, pos=(0.189,−0.098,0.021), debug 세션 20260720_215634)을 대예산 ground-truth 로 이중 판정 (production 예산 40 vs 2000 restarts, raw URDF·zero seed 근사):

| 판정 | 수 |
|---|---:|
| production(40) 통과 | 1/52 (tilt+30° 1가족 — 실물에선 캘 URDF/seed 차이로 이것도 실패) |
| **false negative (해 있는데 솔버가 놓침)** | **0/52** |
| 진짜 도달불가 (2000 restarts 도 실패, 전부 첫 standoff 8cm 에서) | 51/52 |

**결론:** ① 이 전멸은 **솔버 문제가 아니라 워크스페이스 문제** — 로봇팀 소견("손목이 길어 워크스페이스 불리")과 일치. 그 위치(x0.19/y−0.10/z0.02)의 현 파지 가족 설계로는 물리적으로 못 감. 특히 **standoff(접근축 후퇴 8cm)가 파지 pose 보다 먼저 죽는다** — 먼 물체일수록 후퇴 자세가 더 밖으로 나감. ② **CT 문제는 별개로 실재** — "안 된다"는 결론에 27s(감사에서도 실패 판정에만 62s). 수치 IK 는 실패를 증명 못 해 예산을 다 태워야 함. ③ **방향 결정: 해석적 IK 도입** — [EAIK](https://github.com/OstermD/EAIK) (pip, URDF 직접, subproblem decomposition, 전 branch 열거, TUM 논문 기반) 1순위 / [ssik](https://github.com/personalrobotics/ssik) 차선. 기대치 정직하게: **성공률을 올리는 게 아니라**(도달불가는 도달불가) "실패를 ms 에 확정 + walk/restart/예산 노브 통삭제 + 워크스페이스를 수학적 사실로 매핑". 성공률 레버는 후보 가족 설계·물체 배치(워크스페이스 히트맵으로 가시화)가 별도 축. 캘 URDF 는 EAIK(nominal) → 수치 refine(§10.E) polish 로 흡수 — 오늘 refine 은 그 부품으로 재배치.

**후속 정정 (사용자 물리 반박 → 재감사):** "토크오프로 손으로 움직이면 왠만한 위치 다 잡힌다" — 도구가 '불가'래도 물리가 '가능'이면 도구(여기선 **후보 생성**)를 의심. 재감사 결과 **진범 확정**: 물체가 **둥근 큐브**(yaw 물리 자유, 실 ~21×21mm)인데 그 뷰의 관측 footprint 가 노이즈로 20.9×14.9mm = **aspect 1.397 > 문턱 1.25** → yaw 탈출구(`yaw_free` 확장)가 **침묵 미실행** → 52가족 전부 노이즈 낀 OBB yaw(74°/164°) 2방향에 고정 → 그 yaw 로는 진짜 도달불가 → 전멸. **확장 yaw 였으면 3가족 도달 가능이 실측** (ik_yaw_free_audit). 즉 "그 52개 pose 도달불가"(감사)와 "그 물체는 잡을 수 있다"(사용자)가 동시에 참 — 07-17 yaw 복권 사고를 고친 탈출구의 **스위치가 또 노이즈 스칼라**였던 한 단계 위 재발 (침묵 fallback 클래스). 단 그 위치는 3/156 만 통과하는 워크스페이스 가장자리인 것도 사실 (flip/1° 차이로 성패 갈림). → 근본 수술 §11.

### §11. IK·후보생성 대수술 (2026-07-20 밤 구현 완료 — sim 전 계층 통과, 실물 검증만 남음)

> **사용자 지시: "땜빵 금지, 근본 수술" + 모션 계열 대수술 위임.** 수치 IK 의
> "해를 놓쳐도 증명 못 함" 결함을 둘러싼 보상 기계(walk/restart 200/예산
> 노브)와, 연속 파지 조건을 노이즈 앵커 격자로 이산화한 후보 생성(OBB yaw
> 2방향 + aspect 문턱 + 2단 resolve)을 통째로 교체.

**A. 해석적 IK 코어 — [adapters/analytic.py](../backend/modules/motion/adapters/analytic.py) (EAIK 기반, optional)**
- so101 실측 구조: J1(z)⊥**J2∥J3∥J4**(y)⊥J5(z)⊥J6(x, J5 와 교차 0.0mm) = **UR 클래스** (EAIK 판정 `6R-THREE_INNER_PARALLEL`). nominal URDF 의 J6 축이 0.51° 스큐 → **축 snap**(최근접 좌표축) 후 분해.
- 역할 분담: **해석해 = seed 생성기** (완전성 — 모든 branch ≤8 열거, snap 모델이라 실 모델 대비 ~1-2mm), **정밀도 = polish** (§10.E conditional refine 이 캘 적용 실 모델에서 회수). LS(최소자승) 해도 seed 로 수용 — snap 모델은 실 pose 를 '정확히'는 못 맞춰 정상 해가 LS 로 분류됨.
- **branch 는 리밋 clamp (기각 아님)**: 진짜 해가 리밋 경계에 있으면 snap 오차가 경계 밖으로 밀어냄 — clamp 후 polish 가 안으로 회수 (winner-debug 실측: J4=1.61 vs 리밋 1.52 의 정답 branch 를 리밋 필터가 버리던 사례).
- **optional 경계**: EAIK 미설치/분해불가(비-Pieper, omx 등)/snap>5° 면 `try_build`→None → 기존 수치 경로(walk+restart) 폴백. 부팅 로그 1줄로 모드 명시 (`IK=해석적(...)+polish` vs `IK=수치(walk+restart)`) — 침묵 폴백 금지. **EAIK aarch64 wheel 존재 확인** — pi_hori1 은 `uv sync` 로 끝 (소스빌드 불필요). pyproject: pi-hori1 + pc 그룹.
- [pybullet.py](../backend/modules/motion/adapters/pybullet.py) `ik()`: quat 지정 + 해석기 활성이면 `_ik_analytic`(branch 열거 → seed 최근접순 polish+게이트+충돌+**자세잔차 5° 검증** → 첫 통과) — walk/restart 를 안 탄다. **전 branch 실패 = 도달불가 수 ms 확정** (수치의 "못 찾은 걸 수도"가 없음). quat=None(위치-only)은 수치 seeded 유지 (넓은 basin).

**B. 후보 생성 — 절대 yaw 격자 ([servo.py](../backend/modules/tasks/pick_and_place/servo.py) `grasp_families`)**
- **삭제**: `_JAW_YAW_OFFSETS_DEG`(OBB 2방향) / `_YAW_FREE_ASPECT_MAX`(1.25 문턱) / `_YAW_FREE_EXTRA_OFFSETS_DEG` / `yaw_free` 파라미터 / plan·pick 의 2단 resolve 슬라이스.
- **신규**: yaw = 절대 0..180° 15° 격자 + 면 정렬각 2개(grasp_yaw, +90°) 정확값 — **어떤 footprint 든 항상 전 커버** (aspect 안 봄 = 이산화 복권 클래스 소멸). 순서 = tilt 사다리 → 면 정렬 근접 → 짧은변 물기 우선(+90 tie-break, 옛 jaw∥short 1순위 보존) → flip. `GraspFamily.flip` 필드 추가 (refit 변형 매칭 키 — label 은 이제 절대 yaw 포함이라 label 매칭 폐기).
- **직사각 물체의 "긴 변 물기" 차단 = width 물리 게이트** ([plan.py](../backend/modules/tasks/pick_and_place/steps/plan.py) `servo_ladder_groups`): 그 yaw 방향 **관측 폭**(width_along) > 개구(_PICK_MAX_WIDTH_M)면 가족 제외 — aspect 추정이 아니라 점군 실측이 판정.
- `refit_family`: 같은 (tilt, flip) 의 면 정렬 1순위 가족 재유도 (semantics 보존, yaw 차<10° 면 None 유지).

**C. 관측성 (실물 "데이터 없음" 재발 방지)**
- resolve 계약에 `group_failures: list[str]` (그룹별 기각 사유 — "IK pose#0"/"바닥 충돌"/"미탐"...) + `_solve` 가 실패 pose index 반환.
- plan_pick 전멸 시 **tilt×사유 히스토그램** 로그 (`_fail_histogram` — 프리뷰 인덱서 노이즈 방지로 모듈 레벨).
- 기존 §10.E 로깅 유지: resolve 채택 잔차 / MoveL 진단 IK 잔차 max.
- **토크오프 시연 캐처** [scripts/teach_capture.py](../backend/scripts/teach_capture.py): zenoh 구독 전용(모션 명령 0) — Enter 마다 tcp_state(관절 rad+TCP pose, 캘 FK) jsonl 적재. 수집 시연 = 수술 합격 판정 ground truth ("손으로 잡은 자세를 새 시스템이 전부 찾는가").

**D. 검증 (sim 으로 증명된 것)**
| 항목 | 결과 |
|---|---|
| ruff / pyright | clean (전체) |
| fast loop | 374 passed |
| sim suite | 83 passed |
| IK 왕복 1910 (나쁜 seed, 무작위 전역 자세) | 실패 0.3%, pos err mean 1.2mm, 4.5ms/solve |
| 해석 branch | mean 6.8 / max 8, 28µs/열거 |
| **0019 전멸 케이스 (production 경로 재생)** | **어제 27.4s 전멸·원인불명 → 채택 1.76s (jaw@74° tilt+30), 도달가능 7/312 가족, 전멸-스캔도 6.6s + 사유 히스토그램** |
| 도달불가 확정 | 21ms (vs 수치 restart 소진 수백 ms~s) |

**E. 실물에 남은 미지수 (정직)**
- 위 재생은 **raw URDF + zero seed** — 실물은 캘 URDF + 실제 팔 seed 라 수치가 다를 수 있음 (방향은 동일 예상). 부팅 로그 `IK=해석적` 확인이 첫 체크.
- pi_hori1 에서 `uv sync --no-default-groups --group pi-hori1` 1회 필요 (eaik 설치). 실패 시 `IK=수치` 폴백으로 뜨고 기존+refine 동작 (안전).
- 실물 파지 성공률/재시도율/CT before-after — 사용자 직접 측정. 전멸 나면 이제 로그에 tilt×사유 히스토그램이 남는다.
- 워크스페이스 히트맵/캐파맵, standoff 사다리 재설계(가장자리에서 파지보다 먼저 죽는 구조)는 **다음 단계** — 토크오프 시연 데이터(teach_capture)와 함께.
