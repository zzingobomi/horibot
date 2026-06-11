# SO-101 6DOF (구축 중)

OMX_F(5DOF)와 dual-arm cooperative task를 수행할 두 번째 로봇. SO-101 follower(원본 5DOF + 그리퍼)에 wrist yaw 1축을 추가해 **6DOF 팔 + 그리퍼**로 만든다. D405 RGBD를 wrist에 장착해 OMX와 동일한 감지/캘리브레이션 파이프라인에 합류시키는 게 목표.

> 상태 (2026-05-29 기준): **주문 완료, 도착 대기**.
> - ✅ Amazon B0GRPJ2Q8F (SO-101 Follower 풀번들) 주문
> - ✅ WowRobo STS3250 C002 (1/345, shoulder용) 주문
> - ⏳ 다음 단계: 도착 후 3D 부품 출력 (wrist yaw mod + D405 마운트) → 조립 → 백엔드 통합 → 캘리브레이션

---

## 1. 왜 SO-101인가

- **부품 단가 낮음**: 표준 Feetech STS3215 6개 + 3D 프린트 프레임으로 follower 1대 \$150~\$220 수준.
- **오픈소스 자산이 풍부**: TheRobotStudio 공식 리포가 STL/STEP/URDF/OnShape 도큐먼트까지 공개.
- **6DOF 개조도 오픈소스**: 커뮤니티가 wrist yaw mod를 STEP/URDF/onshape-to-robot config까지 같이 공개.
- **dual-arm 협업 task**: OMX 단독으론 안 되는 양손 작업(부품 전달, 한 손이 잡고 다른 손이 조작, 양손 어셈블리)이 최종 목표.

OMX와의 비교: OMX는 Dynamixel(XL430/XL330) + OpenRB-150, SO-101은 Feetech STS3215 + Waveshare bus driver. **모터 SDK/프로토콜이 완전히 다름** — backend의 모터 추상화 분리 필요(아래 §6 참조).

---

## 2. 베이스 키트 구매 옵션

3D 프린트 프레임 + STS3215 ×6 + 컨트롤러 + PSU 패키지. 단품 follower 기준.

| 옵션 | 가격 | 컨트롤러 명시 | fastener | TPU 그리퍼 | 비고 |
|------|------|--------------|---------|-----------|------|
| [thebotstore.co — LeRobot SO-101 Follower](https://thebotstore.co/products/lerobot-so-101-follower) | **\$150** (≈20만원) | "1 motor controller" — 모델 불명 ❓ | 명시 없음 ❓ | 명시 없음 ❓ | 최저가. 메일 확인 후 결정 권장 |
| ✅ **[Amazon B0GRPJ2Q8F — SO-101 Follower Arm Kit](https://www.amazon.com/SO-101-Follower-Arm-Kit-Electronics/dp/B0GRPJ2Q8F)** ⭐ **구매 완료 (2026-05-29)** | **≈30만원** | **Waveshare bus servo driver board** ✅ | 일체 포함 ✅ | TPU compliant gripper ✅ | 구성 가장 투명. Amazon 반품 정책 |
| [Seeed Studio SO-ARM101 듀얼](https://www.seeedstudio.com/SO-ARM101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) / AliExpress | \$220~240 (≈30~33만원) | Waveshare | 포함 | 포함 | **leader+follower 페어**. follower만 필요해도 가성비 좋음. 나중에 imitation learning 데이터 수집(teleop) 가능 |

### 보조 구매 (어디 키트를 사든 별도)

| 항목 | 가격 (대략) | 출처 |
|------|------------|------|
| ✅ **Feetech STS3250 × 1, 1/345 (C002) 기어비** ⭐ **구매 완료 (2026-05-29)** | \$50~70 | **[WowRobo STS3250 C002 — 12V 50KG 1/345 Servo](https://shop.wowrobo.com/products/feetech-sts3250-c002-servo-12v-50kg-1-345-servo)** 에서 주문 |
| **DIN 912 M3x6 또는 M3x8 × 2** (D405 마운트용) | 동네 철물 / 키트 잔여분 | — |
| Intel RealSense D405 본체 | 별도 보유 (OMX와 공유 또는 추가 1대) | [Intel 공식](https://www.intelrealsense.com/depth-camera-d405/) |
| 필라멘트 (PLA+ wrist yaw + D405 마운트) | \$5 안팎 | — |

> ⭐ **왜 STS3215가 아니라 STS3250 1개를 사는가**: 어차피 6DOF mod에 모터 1개 추가가 필요한데, **STS3215(\$30) 대신 STS3250(\$60)을 사면 \$30 차액에 shoulder 토크 한계까지 같이 해결**된다. STS3250은 STS3215와 외형 45.22×24.72×35mm 동일, Double Shaft·TTL 프로토콜·12V·커넥터 모두 동일한 drop-in 호환. 모터 배치는 §3 참조.
>
> **기어비 매칭 주의 ⚠️**: SO-101 키트의 STS3215는 모두 **1/345 (C018)**. STS3250도 반드시 같은 **1/345 기어비 (C002 variant)**로 주문. 다른 기어비면 그 모터만 raw↔rad 변환 계수가 달라져서 `motors.yaml`/`units.py`에서 모터별 분기 필요.
>
> **헷갈리는 variant 비교**:
> - ✅ **STS3250 C002 (1/345, 12V, 50kg·cm)** — 목표
> - ⚠️ STS3250 C001 (Feetech 공식 기본, 기어비 페이지에 없음 — 셀러 확인 필수)
> - ❌ STS3215 C018 (1/345, 30kg·cm) — 키트에 이미 있음, 업그레이드 효과 0
> - ❌ STS3215 C044 (1/191) — leader용 기어비, follower 부적합
> - ❌ STS3215 C001 (7.4V, 19.5kg·cm) — 전압 다름
> - ⚠️ STS3250M (Amazon) — "M" 코어리스 변형, 별도 확인 필요

### 최종 예산

- thebotstore 베이스: **≈30만원** (메일 확인 후, STS3250 포함)
- Amazon 베이스: **≈38만원** (구성 가장 안전, STS3250 포함)
- Seeed 듀얼 베이스: **≈38만원에 leader까지 동봉** ⭐ 향후 데이터 수집 옵션 열어둠 (STS3250 포함)

---

## 3. 6DOF 개조 — ts_flake wrist yaw mod

원본 SO-101은 5DOF 팔 + 그리퍼(모터 6개)로 wrist에 yaw 축이 없다 → 임의 6D 자세 불가, 특정 방향 singular.

**[MakerWorld: SO101 암 + 손목 요(6DoF) by ts_flake](https://makerworld.com/ko/models/1913316-so101-arm-wrist-yaw-6dof)** (2025-10-22 출시, makes 13+, CC BY-NC-SA)

### 필요한 것

- **추가 모터**: Feetech **STS3250 ×1** (50kg·cm, 12V, 1/345 기어비) — shoulder에 박고, 키트의 STS3215 1개를 wrist yaw 슬롯으로 재배치 (§6-5 참조)
- **추가 출력**: 3D 프린트 부품 2개 (주황색 wrist yaw 케이지)
- **출력 시간**: Bambu A1 기준 ~3시간
- **원본 부품**: 그대로 유지 (베이스/숄더/엘보/wrist roll/그리퍼)

### 모터 배치 (총 7개)

| 위치 | 모터 | 출처 |
|------|------|------|
| M1 base yaw | STS3215 | 키트 |
| **M2 shoulder** | **STS3250 (50kg·cm)** ⭐ | 별도 구매 |
| M3 elbow | STS3215 | 키트 |
| M4 wrist pitch | STS3215 | 키트 |
| **M5 wrist yaw (NEW)** | STS3215 | 키트 (원래 shoulder 자리에 갈 모터를 여기로) |
| M6 wrist roll | STS3215 | 키트 |
| M7 gripper | STS3215 | 키트 |

키트 6× STS3215 + 별도 1× STS3250 = 7개. ID 부여는 조립 시 Feetech 툴로 임의 지정.

### 결과 kinematics

원본 5DOF에서 wrist_yaw 1축이 wrist_pitch와 wrist_roll 사이에 삽입됨:

```
base_yaw → shoulder_pitch → elbow_pitch → wrist_pitch → [wrist_yaw NEW] → wrist_roll → gripper
   J1         J2              J3            J4            J5              J6         J7
```

→ **6DOF 팔 + 1DOF 그리퍼 = 모터 7개 총합**.

### CAD 자산도 ts_flake가 공개

**[github.com/ts-flake/d2lrobot](https://github.com/ts-flake/d2lrobot)** (Apache 2.0)

| 자산 | 경로 |
|------|------|
| URDF | [`sim/onshape_to_robot/so101_yaw/robot.urdf`](https://github.com/ts-flake/d2lrobot/blob/main/sim/onshape_to_robot/so101_yaw/robot.urdf) |
| onshape-to-robot config | [`sim/onshape_to_robot/so101_yaw/config.json`](https://github.com/ts-flake/d2lrobot/blob/main/sim/onshape_to_robot/so101_yaw/config.json) |
| convex decomposition 메시 | `sim/onshape_to_robot/so101_yaw/assets/` |
| OnShape 도큐먼트 (ts_flake의 fork) | `5c7655f7a10be258b255653e/w/9b7cf7340f5f742e2d847e84` |

URDF 구조 (실제 확인):
- 7개 revolute joint, axis z 통일, effort 3.3 / velocity 5
- joint 이름은 `joint1`~`joint7` (의미 없음 → semantic 이름으로 rename 권장)
- joint7은 **gripper도 revolute** (4-bar 링케이지로 prismatic 동작 구현)

### URDF 사용 시 주의 ⚠️

1. **mesh 경로가 절대경로**: `/home/dhan/Desktop/study/...`로 박혀있음 (onshape-to-robot이 작성자 로컬 경로 박은 결과). 사용 전 일괄 치환 필요.
2. **카메라가 D405가 아님**: `camera_uvc_25x32.stl`(LeRobot 공식 작은 UVC 카메라). D405 사용 시 mesh + mass + camera_mount origin 모두 갱신 필요(§5).

---

## 4. SO-ARM100/101 공식 리포 자산

**[github.com/TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)**

| 자산 | 경로 |
|------|------|
| **공식 URDF (5DOF, 캘리브레이션 반영)** | [`Simulation/SO101/so101_new_calib.urdf`](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf) |
| 개별 STL (Follower) | [`STL/SO101/Follower/`](https://github.com/TheRobotStudio/SO-ARM100/tree/main/STL/SO101/Follower) |
| 개별 STL (Individual) | [`STL/SO101/Individual/`](https://github.com/TheRobotStudio/SO-ARM100/tree/main/STL/SO101/Individual) |
| OnShape 공식 도큐먼트 | [cad.onshape.com/.../7715cc28...](https://cad.onshape.com/documents/7715cc284bb430fe6dab4ffd/w/4fd0791b683777b02f8d975a/e/826c553ede3b7592eb9ca800) |
| **D405 wrist mount** | [`Optional/Wrist_Cam_Mount_RealSense_D405/`](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Wrist_Cam_Mount_RealSense_D405) |

> OnShape fork 권한: [Issue #147 (Question About Copy Permission for This Onshape Document)](https://github.com/TheRobotStudio/SO-ARM100/issues/147) — 사용 전 확인.

---

## 5. D405 카메라 마운트 — 공식 자산 사용

**[Optional/Wrist_Cam_Mount_RealSense_D405/](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Wrist_Cam_Mount_RealSense_D405)**

| 파일 | 의미 |
|------|------|
| `Wrist_Roll_D405_Holder.stl` | 출력 즉시 사용 |
| `Wrist_Roll_D405_Holder.step` | OnShape/Fusion에서 그대로 통합 가능 |
| `README.md` | 조립: **DIN 912 M3x6 또는 M3x8 스크류 ×2** |
| `d405_mount*.jpg` | 마운트 외관 / 출력 방향 / 샘플 시점 (3장) |

### mod와의 호환성

ts_flake mod의 wrist yaw는 **wrist roll 앞단에 삽입**되는 구조 (URDF에서 joint5가 joint6 wrist_roll보다 위). D405 마운트는 wrist roll 부품 자체에 볼트로 고정 → **mod 적용 후에도 wrist roll 부품 자체는 그대로**라서 물리 충돌 없음.

→ **Fusion 360으로 직접 모델링할 필요 없음**. STEP 그대로 사용.

---

## 6. 백엔드 통합 — 작업 포인트

실물 도착 + 조립 후 진행. OMX 코드 경로와 분리해 namespace 깨끗하게 유지.

### 6-1. 모터 SDK 추상화

현재 [backend/modules/motor/](../backend/modules/motor/)와 `pi-motor` 의존성 그룹은 `dynamixel-sdk` 전제. SO-101 통합 시:

- **Feetech STS 드라이버**(`scservo_sdk` 또는 `feetech-servo-sdk`)를 동등한 위치로 추가
- servo backend 인터페이스 분리 (Dynamixel/Feetech 둘 다 받는 추상 레이어)
- [backend/core/units.py](../backend/core/units.py)의 raw 범위(0..4095, center 2048)는 STS3215도 동일(12-bit) — `units` 자체는 robot-id로 일반화
- **sts3215 / sts3250 mixed bus 지원**: LeRobot이 같은 `FeetechMotorsBus`로 두 모델을 한 버스에서 다루는 패턴을 공식 문서화([예시 코드](https://huggingface.co/docs/lerobot/en/integrate_hardware)). 우리도 motors.yaml에서 슬롯별 `model` 필드만 분기 (M2=`sts3250`, 나머지=`sts3215`).

### 6-2. URDF 배치

- `robot/urdf/so101_6dof/so101_6dof.urdf` 신규 디렉토리
- ts_flake URDF 다운 → mesh 경로 일괄 치환 (`/home/dhan/...` → 상대 경로)
- joint 이름 semantic 변환 (`joint1` → `base_yaw` 등) — 선택 사항
- 카메라 부분 패치:
  - `camera_body` mesh: `camera_uvc_25x32.stl` → `Wrist_Roll_D405_Holder.stl` + D405 본체 STL
  - `camera_body` mass: 0.025 → 0.075 (75g)
  - `camera_mount_joint` origin: D405 마운트 실제 부착 위치로 보정 (STEP에서 측정)

### 6-3. 모터 config

- `robot/config/motors.yaml`에 SO-101용 7개 모터 정의 추가 (또는 robot별 분리 파일)
- USB 포트는 Waveshare bus servo driver 기준 `/dev/ttyACM*` 또는 `/dev/ttyUSB*` (보드 칩셋에 따라)

### 6-4. 공유 싱글톤 일반화

OMX 1대 가정으로 짜인 곳들:
- [backend/core/cache/joint_state_cache.py](../backend/core/cache/joint_state_cache.py) — robot_id 차원 추가 검토
- [backend/modules/kinematics/registry.py](../backend/modules/kinematics/registry.py) — `Kinematics`를 robot별 인스턴스로
- `*Coordinates` 싱글톤 (Joint/Link/Sag) — robot별 npz 분리

> ⚠️ **단, 두 번째 로봇이 아직 물리적으로 존재하지 않음**. 지금 미리 over-generalize하지 말고, 실물 통합 시점에 자연스러운 분기점에서 robot_id 차원 도입.

### 6-5. 하중 분석 — shoulder가 병목, STS3250으로 사전 해결

추가 무게(mod + D405): ≈180g (STS3250 ×1 ≈75g + 3D 부품 30g + D405 75g + 마운트 20g — 키트 STS3215를 wrist yaw로 재배치하지만 모터 자체는 어차피 추가됨).

정적 토크 분석 (full extension, STS3215만 썼을 때):

| 관절 | 부담 토크 (대략) | STS3215 30 kg·cm 대비 |
|------|----------------|---------------------|
| Shoulder (joint2) | ~27~30 kg·cm | **거의 정격 한계** ⚠️ |
| Elbow (joint3) | ~15 kg·cm | 50% |
| Wrist pitch (joint4) | ~5 kg·cm | 여유 |
| Wrist yaw (NEW) | <2 kg·cm | 무부하 수준 |

**병목은 shoulder**. wrist yaw 추가가 아니라 shoulder가 sag/payload 한계를 결정.

**해결: STS3250을 shoulder에 사전 배치** (§2 보조 구매, §3 모터 배치 참조). 50 kg·cm로 67% margin 확보 → sag 크게 감소, payload 한계 stock 수준 회복.

**STS3250 drop-in 호환성 검증 — 공식 + 커뮤니티 근거**:

| 근거 | 출처 |
|------|------|
| **LeRobot 공식 문서가 mixed 패턴 코드로 명시** — `joint_1=sts3250 + joint_2~5=sts3215`를 같은 `FeetechMotorsBus`에 박는 예시 코드 | [HF LeRobot "Bring Your Own Hardware"](https://huggingface.co/docs/lerobot/en/integrate_hardware) |
| **공식 지원 모터 리스트에 sts3250 포함** — "STS & SMS series (protocol 0): sts3215, sts3250" 같은 드라이버에서 처리 | 동일 문서 |
| **외형 동일** — STS3250: 45.22×24.72×35mm vs STS3215: 45.2×24.7×35mm (0.02mm 반올림 차이) | [Feetech 공식](https://www.feetechrc.com/sts_ttl_series%20servo.html) |
| **커뮤니티 실증 영상** | [YouTube Short: SO-ARM101 + STS3250](https://www.youtube.com/shorts/qb2OYatvPWU) |

- Double Shaft 동일, TTL 프로토콜 동일, 12V 동일, 커넥터 동일
- 3D 프린트 케이지 변경 없이 그대로 들어감 (form factor 일치 가정)
- 무게만 +4.5g (shoulder 위치라 distal joint torque에 영향 미미)

⚠️ TheRobotStudio 공식 SO-ARM 리포에는 STS3250 빌드 가이드가 없으므로, **구매 전 [LeRobot Discord](https://discord.gg/s3KuuzsPFb) `#so-arm` 채널에 최종 확인** 권장 ("Anyone swapped STS3215 → STS3250 on SO-101 follower joint 2 without case modification?").

**보조 운영 가이드** (STS3250 적용 후에도 권장):
- 팔 수평 장시간 펼침 금지 (모터 발열)
- 무거운 물체는 elbow 굽힌 자세에서 들기

캘리브레이션(OMX의 확장 BA + sag 모델)은 그대로 적용해서 잔여 sag/오차 흡수.

### 6-6. Feetech 모터 provisioning (ID / PID)

OMX(Dynamixel) 와 절차가 두 군데에서 갈림 — SO-101 도착 직후 1회성으로 정리해야 backend 통합 진입 가능.

#### (a) 모터 ID 굽기 — 출하 시 전부 ID=1

STS3215/STS3250 은 공장 출하 시 모든 모터가 ID=1. 그대로 버스에 다 물리면 충돌 → 한 개씩 연결해서 ID 부여 후 다음 모터 추가. 세 가지 길:

| 방식 | 환경 | 마찰 | 비고 |
|------|------|------|------|
| **FD (FT SCServo Debug Software) GUI** ⭐ | Windows | 가장 낮음 | Dynamixel Wizard 와 동일 UX. scan → 모터 클릭 → ID/baud 필드 직접 편집. 1회성 provisioning 에 권장 |
| `lerobot setup_motors` script | Linux/Mac/Win | 중 | SO-ARM 커뮤니티 표준 walk-through. lerobot 의존성 깔아야 함 (provisioning 머신에만, 우리 backend 엔 안 들어감) |
| 자체 CLI (`backend/scripts/feetech_set_id.py`) | 어디서나 | 중~높음 | `scservo_sdk` 직접 호출. 30줄 정도. 우리 motors.yaml 의 ID 배치와 1:1 매칭되는 인자 받음. study 단계 밟기 의식할 때 |

→ **1회만 쓰는 작업이라 FD GUI 가 합리적**. 학습 의식하면 lerobot script 한 번 돌려보고 우리 layout 에 맞게 자체 CLI 도 짜기.

모터별 ID 배치는 §3 표 (M1=1, M2=2, …, M7=7).

#### (b) PID 튜닝 — Dynamixel(RAM) vs STS(EEPROM)

| 항목 | Dynamixel X | Feetech STS |
|------|------------|-------------|
| Position P/I/D 위치 | **RAM** (주소 80/82/84) | **EEPROM** (주소 21/23/22 부근) |
| 전원 사이클 후 | **기본값으로 리셋** | **유지** |
| backend init 시 처리 | 매번 write 필수 (현재 패턴) | read → diff → 다르면 write |
| 게인 구조 | cascaded (position P/I/D + velocity P/I) | 단일 position loop 만 |
| 해상도 | 16-bit | 8-bit (0–255) |
| 동적 게인 스케줄링 (자세별 등) | OK | 비권장 (EEPROM wear) |

→ STS 는 한 번 굽고 끝나는 정적 튜닝 모델. 우리 use case (정해진 payload, dual-arm pick&place) 엔 오히려 마찰 적음.

#### (c) Backend init 에서의 동기화 패턴

motors.yaml 에 PID 값까지 명시하고 (Dynamixel 처럼 SSOT 유지), backend start 에서 동기화. 다만 STS 는 EEPROM wear 방지 위해 **read-first-then-write**:

```python
def ensure_pid(motor_id, kp, ki, kd):
    cur = read_pid(motor_id)                # (kp, ki, kd) tuple
    if cur == (kp, ki, kd):
        return                               # 일치하면 write 0 회
    unlock_eeprom(motor_id)                 # Write_Lock=0 (주소 55 부근)
    write_pid(motor_id, kp, ki, kd)
    lock_eeprom(motor_id)                   # Write_Lock=1
```

핵심:

- **EEPROM lock register 풀고 잠그기 필수** — Dynamixel 의 `Torque_Enable=0 → EEPROM 영역 write` 패턴과 비슷한데 STS 는 별도 flag.
- **읽고 비교부터** — backend init 100 번 돌려도 motors.yaml 안 바뀌면 write 0 회. 평생 write 수십 회 수준이라 EEPROM 한도 (수만~십만 회) 한참 못 미침.
- **Backend adapter 인터페이스 통일** — `backend.set_pid(motor_id, gains)` 한 메서드로 노출, 내부에서 Dynamixel(매번 write) / STS(read-diff-write) 알아서 분기. 호출자는 두 로봇 동일하게 다룸.

#### (d) 동적 튜닝 필요 시

자세별 게인 스케줄링 같은 게 정말 필요해지면 STS 에선 EEPROM 으로 못 함. 그땐 별도 검토:

- velocity / current 모드로 전환 후 우리 쪽에서 outer position loop 짜기
- 또는 Dynamixel 로 교체

지금 단계엔 가능성만 메모. **default 게인 → shoulder sag/진동 본 후 한 번 튜닝 → 끝** 으로 가는 게 SO-101 답.

### 6-7. 캘리브레이션 산출물

OMX와 동일한 4종 + intrinsic 체계를 SO-101용으로 별도 산출:
- `robot/calibration/so101_intrinsic.npz` (D405 공유 가능 — 카메라가 같으면)
- `robot/calibration/so101_hand_eye.npz`
- `robot/calibration/so101_joint_offset.npz`
- `robot/calibration/so101_link_offset.npz`
- `robot/calibration/so101_sag_offset.npz`

OMX에서 검증된 확장 BA + 물리 sag 모델([docs/hand_eye_extended_ba.md](hand_eye_extended_ba.md)) 그대로 적용. SO-101의 STS3215 stiffness 측정값으로 sag 모델 재학습.

---

## 7. 전원 토폴로지 — 벤치 파워 swap 결정

### 결론

| 로봇 | 전원 | 비고 |
|------|------|------|
| **OMX_F** | SO-101 키트 12V 5A 어댑터 (swap 후) | 현재 벤치 파워(악어집게) → 키트 어댑터로 다운그레이드 |
| **SO-101 (6DOF mod)** | 벤치 파워 12V (전류 limit 6A, 악어집게) | 현재 OMX 의 벤치 파워를 그대로 이전 |

> ⚠️ **이 결정은 SO-101 도착 직전(또는 직후) 실행. 현재 상태는 OMX = 벤치, SO-101 = 미도착.**

### 배경 — 왜 swap 이 필요한가

SO-101 키트 어댑터는 **원래 5DOF + 그리퍼 (STS3215 ×6)** 기준 산정. 너의 6DOF mod 는:

- 모터 **6 → 7** 개 (wrist yaw 추가)
- shoulder 가 STS3215 (30kg·cm) → **STS3250 (50kg·cm)**

→ 동시 동작 피크 전류 추정 **~4.5~5A**, 키트 5A 어댑터 마진 **0~10% (빡빡)**. 정상 운영은 가능해도 피크에서 limit 걸리면:
- 모터 리셋 / trajectory 미스 / `motor torque drop`
- 디버깅 시 transparent 하게 안 보임 (어댑터엔 전류계 없음)

벤치 파워 (12V/limit 6A) 로 운영하면:
- 마진 25% 확보 (limit 6A vs 피크 5A)
- **실시간 전류 가시화** — STS3250 효과 / 자세별 부하 / wrist yaw 추가 영향 모두 측정 가능 (study output)

OMX 는 모터가 작아 (XL430 ×3 + XL330 ×3 강압 5V) 키트 5A 어댑터로 충분.

### 트레이드오프 — 검토한 옵션 3개

| 옵션 | 구성 | 추가 비용 | 채택 여부 | 이유 |
|------|------|----------|---------|------|
| **A. swap** ⭐ | OMX → 키트 / SO-101 → 벤치 | 0원 | **채택** | 비용 0 + study 가시화 + OMX 충분 |
| B. 추가 PSU 구매 | 둘 다 자기 전원 (SO-101 용 12V 8~10A 신규) | 2~3만원 | 보류 | OMX 가 이미 안정되어 추가 디버깅 필요성 낮음 — ROI 불명확 |
| C. 일단 키트로 시작 | OMX → 벤치 / SO-101 → 키트 | 0원 | 기각 | 피크 부족 시 디버깅 지옥 (gauge 없음, transparent X) |

### 실행 절차

**선결 확인 (다음 세션 하드웨어 사진으로 검증):**

- [ ] **OpenRB-150 배럴잭 입력 소켓 유무** — 현재 사진엔 녹색 터미널 블록만 보임. 보드 옆면/다른 면에 배럴잭 있는지
- [ ] **SO-101 키트 어댑터 배럴잭 사이즈** — 보통 5.5×2.1mm, center positive
- [ ] **SO-101 컨트롤러 보드 (Waveshare) 전원 입력 형식** — 배럴잭 / USB-C PD / 터미널 블록 중 어느 것

**Case 1: OpenRB-150 에 배럴잭 입력 있고 사이즈 맞음 (가장 깔끔)**

1. OMX 벤치 파워 OFF → 악어집게 떼기
2. SO-101 키트 어댑터를 OpenRB-150 배럴잭에 그대로 꽂기
3. OMX 전원 ON → idle 동작 확인 (모터 LED / move_j 부드러움)
4. SO-101 도착 후 → SO-101 컨트롤러 보드 전원 입력에 벤치 파워 악어집게 결선 (잭 안쪽 핀 = +, 바깥 쉘 = -)

**Case 2: 배럴잭 없거나 사이즈 안 맞음**

1. **DC 배럴잭 to 베어 와이어 어댑터** (다이소/쿠팡 1~2천원) 추가 구매 *or* **5.5×2.1mm 암 panel mount 잭 (3핀)** 보유 시 직접 빨/검 와이어 납땜 *or* 키트 어댑터 잭 자르고 빨/검 노출
2. OMX 의 OpenRB-150 녹색 터미널 블록에 + / - 결선 (현재 악어집게 자리와 동일 위치)
3. 나머지 Case 1 과 동일

> 직접 결선 시 멀티미터 검증 절차는 바로 아래 "Case 2 보조" 참조.

### Case 2 보조 — 암 panel mount 잭 직접 결선 + 멀티미터 검증 (첫 사용자용)

5.5×2.1mm 암 panel mount 잭 (3핀짜리) 보유 시 사전 제작 어댑터 대신 직접 빨/검 와이어 결선. 멀티미터 처음 쓰는 경우 참고.

> ⚠️ 3핀 = Center (+) / Sleeve (−) / Switch (NC). 메이커마다 러그 배치 다르므로 **눈으로 식별 X, 멀티미터로 검증 필수**.

**멀티미터 사전 세팅:**

| 항목 | 설정 |
|------|------|
| 검정 프로브 | **COM** 단자에 꽂음 (항상 고정) |
| 빨강 프로브 | **VΩmA** 또는 **V** 단자에 꽂음 |
| 도통 검사 다이얼 | 🔊 or ))) (음파/스피커 표시) |
| 전압 측정 다이얼 | **V⎓** 또는 **V—** (직류, 20V 레인지 or 자동) |

> 다이얼의 V~ (물결) = AC, V⎓ (직선) = DC. 우리는 12V DC.

---

**1단계: 잭 3핀 중 +/- 식별 (도통 모드)**

🚨 이 단계엔 **어댑터를 콘센트에 꽂지 않음**. male 플러그만 빌려쓰는 용도.

1. 멀티미터 다이얼을 🔊 도통 모드로 → 프로브 두 끝을 서로 맞대 **"삐~"** 울리는지 확인 (정상 작동 확인)
2. SO-101 키트 12V 어댑터의 male 플러그를 암 잭에 **끝까지 꽂음** (switch 핀을 sleeve 에서 분리시키기 위해 필수)
3. **+ 핀 식별:**
   - 검정 프로브를 male 플러그의 **안쪽 작은 핀 끝**에 갖다 댐 (어댑터 코드 쪽 케이블 옆에 노출된 부분)
   - 빨강 프로브로 잭 뒷면 러그 3개를 **하나씩** 찍어봄
   - **"삐" 울리는 러그 = Center (+)** → 매직펜으로 점 표시
4. **− 핀 식별:**
   - 검정 프로브를 male 플러그의 **바깥 금속 배럴** (은색 원통 부분) 에 갖다 댐
   - 빨강 프로브로 남은 2개 러그 찍어봄
   - **"삐" 울리는 러그 = Sleeve (−)** → 다른 색 표시
5. 남은 1개 = Switch → **사용 안 함** (와이어 안 연결)

---

**2단계: 와이어 임시 결선 + polarity 검증 (납땜 전, DC 전압 모드)**

1. 빨강 와이어 끝을 + 러그에, 검정 와이어 끝을 − 러그에 **손가락 / 집게 클립으로 꾹 누름** (와이어끼리 안 닿게 주의)
2. 어댑터를 콘센트에 꽂음 (어댑터 LED 점등 확인)
3. 멀티미터 다이얼을 **V⎓ 20V** (또는 자동 레인지) 로 변경
4. **빨강 프로브 → 빨강 와이어 끝** / **검정 프로브 → 검정 와이어 끝** 에 댐
5. 화면 표시 확인:
   - **+12.0V (±0.5V)** = ✅ 통과, 다음 단계로
   - **−12V (음수, 앞에 마이너스)** = 단자 거꾸로 식별. 어댑터 콘센트 분리 후 1단계 재실행
   - **0V** = 와이어가 러그에 안 닿음. 위치/접촉 다시
6. ✅ 통과 시 어댑터를 콘센트에서 빼고 납땜 시작 (인두로 빨/검 와이어를 각자 식별된 러그에 고정)

---

**3단계: 납땜 후 최종 polarity 재검증**

1. 납땜 완료된 잭에 어댑터 male 플러그 꽂음 → 어댑터 콘센트에 꽂음
2. 와이어 반대쪽 끝 (벗겨놓은 노출 부분) 에서 DC 전압 재측정 → **+12V 정극성** 확인
3. ✅ 통과 시 OpenRB-150 녹색 터미널 블록에 결선:
   - **빨강 와이어** → 보드의 **VIN / V+ 핀**
   - **검정 와이어** → 보드의 **GND 핀**
   - (보드 실크스크린 표기 / OpenRB-150 [공식 핀맵](https://docs.arduino.cc/hardware/opta) 으로 핀 위치 확인 필수)

---

**자주 하는 멀티미터 실수:**

| 증상 | 원인 | 해결 |
|------|------|------|
| 도통 모드인데 "삐" 안 울림 | 다이얼이 저항 (Ω) 모드 | 🔊 / ))) 표시 찾기 (보통 Ω 옆 칸) |
| 전압 측정 시 화면 OL / "1." 만 표시 | 검정 프로브가 10A / 전류 단자에 꽂힘 | **COM** 단자로 옮기기 |
| 전압이 이상하게 작게 (~0V) 나옴 | AC 모드 (V~) 로 측정 | **V⎓** (직선) DC 모드로 변경 |
| male 플러그 안쪽 핀에 프로브 안 닿음 | 잭에 꽂힌 상태라 핀 가려짐 | 어댑터 잭에서 살짝 빼서 핀 노출 → 한 손으로 다시 끝까지 꽂은 자세 유지하며 측정 |
| 측정값이 계속 흔들림 | 프로브 접촉 불량 | 프로브 끝 / 와이어 노출부 깨끗한지 확인, 더 꾹 누름 |

---

**테스트 시나리오 (swap 후):**

| 단계 | 기대값 | 실패 시 |
|------|--------|---------|
| OMX idle (torque off) | 모터 LED 정상 점등 | 어댑터 polarity 확인 |
| OMX MOTOR_ENABLE → home pose | 자세 유지 부드러움 | 어댑터 전압 강하 확인 (멀티미터) |
| OMX 전축 동시 move_j | 부하 자세에서도 리셋 없음 | 키트 어댑터 5A 부족 가능성 → 벤치 임시 복귀 후 측정 |

### 운영 시 메모

- **둘 다 동시 측정 불가**: SO-101 디버깅 시 OMX 끄고 벤치 임시 복귀 (악어집게라 옮기기 쉬움). 평소엔 SO-101 전용
- **SO-101 STS3250 baseline 측정 권장**: 도착 직후 idle / home pose / 전축 동시 move / 무부하 + payload 50g, 100g, 200g 별 피크 전류 메모 → OMX 와 직접 비교 데이터 확보
- **OMX hardware.md 의 "11V 복귀 (commit `adec924`)" 기록은 outdated** — 현재 12V 벤치 운영 중. SO-101 swap 작업과 별개로 [hardware.md](hardware.md) §전원 토폴로지 정정 필요 (TODO)

---

## 8. 실행 순서

```
[1] ✅ 베이스 키트 결정 (Amazon B0GRPJ2Q8F 주문 완료 — 2026-05-29)
       ↓
[2] ✅ 부속 주문:
      - STS3250 ×1 (WowRobo C002, 1/345) 주문 완료 — 2026-05-29
      - D405 (별도 보유분 활용 or 신규)
       ↓
[3] ⏳ 출력: wrist yaw mod 2개 부품 + Wrist_Roll_D405_Holder
       ↓
[4] (선택) ts_flake에게 댓글로 두 가지 문의:
    - OnShape 도큐먼트 copy 권한
    - D405 카메라 variant 계획
       ↓
[5] 조립
      - 모터 배치 (§3 표): M2 shoulder = STS3250, M5 wrist yaw = 키트 STS3215
      - Feetech 모터 ID 1~7 굽기 + (필요시) PID 기본값 튜닝 — §6-6 참조
      - yaw mod 설치 → D405 마운트 + 카메라
       ↓
[6] 백엔드 통합 (§6 작업 — 모터 SDK 추상화, URDF 배치, motors.yaml, 캘 산출물)
       ↓
[7] 캘리브레이션 (joint_offset → link_offset → hand_eye → sag → BA refinement)
       ↓
[8] dual-arm task 설계 시작
```

---

## 9. 참고 링크 모음

### 하드웨어 / 구매
- [thebotstore.co SO-101 Follower](https://thebotstore.co/products/lerobot-so-101-follower)
- [Amazon SO-101 Follower Arm Kit (B0GRPJ2Q8F)](https://www.amazon.com/SO-101-Follower-Arm-Kit-Electronics/dp/B0GRPJ2Q8F)
- [Amazon SO-101 Follower Electronics Kit](https://www.amazon.com/SO-101-Follower-Electronics-Feetech-STS3215/dp/B0GH35175P)
- [Seeed Studio SO-ARM101 Pro](https://www.seeedstudio.com/SO-ARM101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html)
- [Waveshare ST3215 Servo Wiki](https://www.waveshare.com/wiki/ST3215_Servo)
- [Intel RealSense D405 공식](https://www.intelrealsense.com/depth-camera-d405/)

### 공식 SO-101 자산
- [TheRobotStudio SO-ARM100 리포](https://github.com/TheRobotStudio/SO-ARM100)
- [공식 URDF (so101_new_calib)](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf)
- [Hugging Face SO-101 공식 문서](https://huggingface.co/docs/lerobot/so101)
- [SO-101 STL Follower 폴더](https://github.com/TheRobotStudio/SO-ARM100/tree/main/STL/SO101/Follower)
- [SO-ARM101 OnShape 공식](https://cad.onshape.com/documents/7715cc284bb430fe6dab4ffd/w/4fd0791b683777b02f8d975a/e/826c553ede3b7592eb9ca800)
- [D405 wrist mount STL+STEP](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Wrist_Cam_Mount_RealSense_D405)
- [Issue #147 OnShape Copy Permission](https://github.com/TheRobotStudio/SO-ARM100/issues/147)

### Wrist yaw 6DOF mod
- [MakerWorld: SO101 암 + 손목 요(6DoF)](https://makerworld.com/ko/models/1913316-so101-arm-wrist-yaw-6dof)
- [ts-flake/d2lrobot GitHub (URDF/CAD)](https://github.com/ts-flake/d2lrobot)
- [ts_flake URDF 직접 링크](https://github.com/ts-flake/d2lrobot/blob/main/sim/onshape_to_robot/so101_yaw/robot.urdf)
- [onshape-to-robot 툴](https://github.com/Rhoban/onshape-to-robot)

### 빌드 가이드 (커뮤니티)
- [arturlab SO-101 빌드 튜토리얼](https://arturhabuda.com/2025/07/01/build-tutorial-so-101-robot-arms/)
- [Seeed Studio Wiki — LeRobot SO10x](https://wiki.seeedstudio.com/lerobot_so100m/)
- [OpenELAB SO-101 빌드 가이드](https://openelab.io/blogs/seeed-studio/build-your-own-so-101-robot)
- [phospho.ai SO-101 Quickstart](https://docs.phospho.ai/so-101/quickstart)

### 관련 OMX 문서
- [hardware.md](hardware.md) — OMX 모터/컨트롤러/전원
- [calibration_workflow.md](calibration_workflow.md) — 캘리브레이션 절차
- [calibration_apply_flow.md](calibration_apply_flow.md) — 4종 캘 산출물 적용 메커니즘
- [hand_eye_extended_ba.md](hand_eye_extended_ba.md) — 확장 BA + sag 모델 (SO-101 캘에 그대로 적용 가능)
