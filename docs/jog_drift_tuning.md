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
- **절차**: [docs/calibration_workflow.md](calibration_workflow.md) — Hand-Eye 캘과 같은 자세 패턴 + sag 계산

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

- [motion_taxonomy.md](motion_taxonomy.md) — 3 계층 motion primitive 분류, Phase 1 채택
- [calibration_workflow.md](calibration_workflow.md) — 4종 캘리브레이션 절차
- [calibration_apply_flow.md](calibration_apply_flow.md) — sag/link/joint offset 적용 메커니즘
- [Ruckig Synchronization](https://github.com/pantor/ruckig) — Phase/Time/TimeIfNecessary/No 옵션
- [Feetech LeRobot motors](https://www.mintlify.com/huggingface/lerobot/motors/feetech) — STS3215 default 패턴
- [LeRobot configure_motor issue #673](https://github.com/huggingface/lerobot/issues/673) — Maximum_Acceleration 설정
