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
- [backend/core/joint_state_cache.py](../backend/core/joint_state_cache.py) — robot_id 차원 추가 검토
- [backend/modules/kinematics/solver.py](../backend/modules/kinematics/solver.py) — `PybulletSolver`를 robot별 인스턴스로
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

### 6-6. 캘리브레이션 산출물

OMX와 동일한 4종 + intrinsic 체계를 SO-101용으로 별도 산출:
- `robot/calibration/so101_intrinsic.npz` (D405 공유 가능 — 카메라가 같으면)
- `robot/calibration/so101_hand_eye.npz`
- `robot/calibration/so101_joint_offset.npz`
- `robot/calibration/so101_link_offset.npz`
- `robot/calibration/so101_sag_offset.npz`

OMX에서 검증된 확장 BA + 물리 sag 모델([docs/hand_eye_extended_ba.md](hand_eye_extended_ba.md)) 그대로 적용. SO-101의 STS3215 stiffness 측정값으로 sag 모델 재학습.

---

## 7. 실행 순서

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
      - Feetech 툴로 모터별 ID 1~7 지정
      - yaw mod 설치 → D405 마운트 + 카메라
       ↓
[6] 백엔드 통합 (§6 작업 — 모터 SDK 추상화, URDF 배치, motors.yaml, 캘 산출물)
       ↓
[7] 캘리브레이션 (joint_offset → link_offset → hand_eye → sag → BA refinement)
       ↓
[8] dual-arm task 설계 시작
```

---

## 8. 참고 링크 모음

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
