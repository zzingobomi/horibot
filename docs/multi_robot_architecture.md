# Multi-Robot Platform Upgrade — Architecture Design

OMX_F 단독 시스템에서 **OMX_F + SO-101 6DOF dual-arm cooperative manipulation 플랫폼** 으로의 전면 architecture 업그레이드 design 문서. 단일 robot 가정으로 짜인 코드베이스 (토픽 namespace / kinematics solver / motor backend / Step DSL / 캘리브레이션) 를 multi-robot 으로 일반화하는 통합 plan.

본 문서는 "design + 마이그레이션 plan" 이지 즉시 구현 명세가 아님. SO-101 도착 시점에 이 문서를 토대로 본격 작업.

**관련 문서:**

- [so101_6dof_plan.md](so101_6dof_plan.md) — 하드웨어 plan + 모터 SDK 추상화 (§6) — 본 문서의 prerequisite
- [step_dsl.md](step_dsl.md) — Step DSL 토대 (Step/Slot/StepContext/Recipe) — multi-robot 확장 base
- [calibration_apply_flow.md](calibration_apply_flow.md) — 4종 캘 적용 메커니즘 — robot 별 분리 대상
- [random_palletizing.md](random_palletizing.md) — task 예시 — multi-robot 으로 확장될 task 의 한 사례

---

## 1. 개요

### 1.1 현재 (OMX-only)

- Robot: OMX_F (5DOF + 그리퍼, Dynamixel XL430/330 + OpenRB-150) 1대
- 토픽 prefix: `omx/*` — 시스템 이름이 robot 종류와 섞임
- Kinematics: `PybulletSolver` 단일 인스턴스
- 캘리브레이션: 5종 산출물 (`robot/calibration/*.npz`) — robot 식별자 없이 단일 set
- Step DSL: "the robot" 가정, robot_id 인자 없음
- Coordination: 개념 없음 (단일 robot 이라 불필요)

### 1.2 미래 (multi-robot cooperative)

- Robots: OMX_F (5DOF) + SO-101 6DOF (Feetech STS3215/3250 + Waveshare) 2대
- 공조 패턴: bimanual / handoff / parallel / lead-follow (cf. so101 plan §1, §10.1)
- 토픽 namespace: robot-scoped + system + coordination 분리
- Kinematics: `IKSolver` Protocol + robot 별 adapter
- 캘리브레이션: robot 별 5종 + **robot-to-robot extrinsic 신규 (7번째)**
- Step DSL: robot_id-aware step + Coordinator 신규
- Coordination: workspace conflict / sync barrier / task allocation layer 신규

### 1.3 핵심 design 원칙

1. **Adapter + Strategy + DIP** — 외부 system 마다 Protocol 정의, 구현체 swap 가능, 호출처는 interface 만 의존. Hexagonal Architecture 의 핵심 원칙 차용하되 layer 강제 / DTO / DI container 같은 무거운 부속은 박지 않음 — multi-robot 추상화에 필요한 핵심 패턴 셋만 적용
2. **점진 마이그레이션** — Phase 1 에서 interface 만 도입 (OMX 만 implement, 동작 변화 0). Phase 2 에서 SO-101 adapter 추가. 한 번에 다 바꾸지 않음
3. **so101_6dof_plan §6.4 "over-generalize 금지" 룰 존중** — interface 는 미리 깔되 robot_id 차원의 실제 사용은 SO-101 도착 시점에 자연스러운 분기점에서 도입
4. **design 은 미리, 구현은 적시에** — 본 문서는 design 미리 정리. 구현은 phase plan 따라

---

## 2. Repo 이름 변경

### 2.1 동기

현재 repo 이름 `omx-control` 은 단일 robot 이름을 박은 것. 두 번째 robot (SO-101) 추가 + cooperation layer first-class 가 되면:

- 이름 misleading — "OMX 만 제어" 처럼 보임
- 신규 contributor / 외부 reference 시점 의도 전달 안 됨
- 디렉토리 / config path 도 이름 영향 받음

### 2.2 결정: `horibot`

**이름**: `horibot`

**어원**: "호리" 는 사용자 아들의 태명. 개인 의미를 담은 작명.

**상태**: ✅ **완료** (commit `1114524 chore: omx-control → horibot 프로젝트 명 갱신`). 절차 / 회고는 §2.4.

### 2.3 시스템 이름 prefix 박지 말기 (★ 중요)

repo 이름이 무엇이든 **토픽 namespace 의 prefix 로 박지 않음**. ROS 관례 (ROS namespace 는 logical hierarchy 만, 시스템 이름 prefix 없음) 그대로. 시스템 이름이 namespace 에 박히면 이름 변경 시 모든 토픽이 깨짐.

### 2.4 변경 절차 (마이그레이션 step) — ✅ 완료

**Rename 으로 진행** — GitHub 의 정식 rename 기능 사용. 새 repo 생성 안 함.

실제 실행 결과는 §2.4 마지막 "완료 체크리스트" 참조. 아래 절차 / 비교 / 회고는 retrospective.

#### Rename vs 새 repo 비교

| 항목                  | **Rename** ⭐           | 새 repo               |
| --------------------- | ----------------------- | --------------------- |
| Git history           | 보존                    | 옮기려면 manual push  |
| Issues / PR           | 보존                    | 옮기기 매우 어려움    |
| Stars / forks         | 보존                    | 옮길 수 없음          |
| 외부 link             | 자동 redirect           | 모두 깨짐             |
| 작업 단위             | rename + remote 갱신    | git push / 정리 다 새로 |
| 심리적 cut-off       | 약함                    | 완전 새 시작 느낌     |

학습 platform 이고 stars / 외부 reference 영향 작아 — **Rename 으로 충분, 새로 팔 이유 없음**.

GitHub rename 의 동작: **old URL 영구 redirect** — `github.com/<user>/old-name` 도 새 이름으로 자동 forward. 외부 link / 기존 `git fetch` `git push` 다 작동 (redirect 따라감). 단 best practice 는 local 의 remote URL 갱신.

#### Phase 1 시작 시점에 일괄 실행 (예시: `omx-control` → `horibot`)

**1. GitHub rename**:

```bash
gh repo rename horibot
# 또는 UI: Settings → Repository name → Rename
```

**2. Local remote URL 갱신** (redirect 작동하지만 best practice):

```bash
git remote set-url origin https://github.com/<user>/horibot.git
git remote -v   # 확인
```

**3. Local directory rename** (선택 — 깔끔하게 가려면):

- Windows: 폴더 우클릭 → rename. 예: `d:\Study\omx\omx-control` → `d:\Study\horibot`
- 또는 같은 path 유지 (외부 영향 X). 단 path 가 이름 misleading 됨
- VSCode workspace: 닫고 새 path 로 다시 open

**4. ★ 메모리 경로 마이그레이션** (★ local directory rename 한 경우에만):

Claude Code 의 메모리는 디렉토리 path 기반 인코딩:

```
d:\Study\omx\omx-control
↓ 인코딩: ':' → '-', '\' → '-'
C:\Users\<user>\.claude\projects\d--Study-omx-omx-control\memory\
```

새 path 도 같은 규칙으로 변환 후 폴더 이동:

```powershell
# 예: d:\Study\horibot 가 새 path 라면 → d--Study-horibot
Move-Item `
  "C:\Users\<user>\.claude\projects\d--Study-omx-omx-control\memory" `
  "C:\Users\<user>\.claude\projects\d--Study-horibot\memory"
```

⚠️ 이거 안 하면 새 path 에서 Claude Code 시작 시 메모리 (`MEMORY.md` + 모든 `feedback_*.md` / `project_*.md` 등) 다 비어보임. 한 번만 옮기면 끝.

**5. README / 문서의 repo URL 박힌 부분** — clone command / 외부 reference URL 정정. internal link 는 대부분 상대 경로 (`docs/...` `backend/...`) 라 path 안 박혀있어 영향 없음

**6. CLAUDE.md** — 첫 줄 프로젝트 명 (`# CLAUDE.md` 다음의 "OMX Control" 같은 명칭) 갱신

**7. CI / external integrations** — GitHub Actions workflow / webhook 등 repo name reference. 대부분 redirect 자동 따라가지만 정정 권장

**8. 외부 dependents** — 없으면 skip. 있으면 갱신 (PyPI 패키지 / 외부 reference 등 — 이 프로젝트는 해당사항 없을 듯)

#### 소요 / Reversibility

- **총 소요**: 30분 이내. 대부분 즉시 / manual
- **Reversible**: GitHub rename 은 다시 rename 으로 되돌릴 수 있음 (원래 이름이 still available 한 경우)
- **메모리 폴더는 Move-Item 이라 backup 권장**: 만약 path 인코딩 실수해서 못 찾으면 backup 에서 복구

#### Trigger 시점

Phase 1 의 **interface 도입 작업 시작 직전** 이 자연스러움 — repo 이름 (예: `horibot`) 으로 새 디렉토리 시작하면 그 시점부터 모든 작업이 새 이름 context. 이전 history 는 GitHub redirect 로 보존.

#### 완료 체크리스트 (commit `1114524`)

| 단계 | 상태 | 비고 |
| ---- | ---- | ---- |
| 1. GitHub rename                              | ✅ | `github.com/zzingobomi/horibot.git` |
| 2. Local remote URL 갱신                      | ✅ | `git remote -v` 로 확인 |
| 3. Local directory rename                     | ✅ | `d:\Study\horibot` |
| 4. 메모리 경로 마이그레이션                   | ✅ | `~/.claude/projects/d--Study-horibot/memory/` |
| 5. README / 문서의 repo URL                   | ✅ | 본 문서 §2 의 historical reference 외 잔존 없음 |
| 6. CLAUDE.md 프로젝트 명                      | ✅ | "Horibot — OMX_F ..." |
| 7. CI / external integrations                 | N/A | CI workflow 없음 |
| 8. 외부 dependents                            | N/A | 없음 |

---

## 3. 핵심 abstraction layers

각 외부 system (PyBullet / Dynamixel / Feetech / D405 등) 을 Python `Protocol` 로 wrap 하는 **Adapter Pattern**. config 로 어느 adapter 쓸지 결정 (**Strategy Pattern**). 호출처는 Protocol 만 의존 (**DIP — Dependency Inversion Principle**). 큰 그림 framing 은 Hexagonal Architecture 와 동일하지만, Spring/DDD 식의 layer 강제 / DTO 변환 / DI container 등 무거운 부속은 박지 않음 — multi-robot 추상화에 필요한 핵심 패턴 셋만 적용.

### 3.1 IKSolver (Protocol)

```python
# backend/modules/kinematics/iksolver.py
from typing import Protocol
import numpy as np

class IKSolver(Protocol):
    def fk(self, joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """joints (n,) → position (3,), quaternion (4,)"""
        ...
    def ik(self, pos: np.ndarray, quat: np.ndarray | None = None,
           seed: np.ndarray | None = None) -> np.ndarray:
        """target pose → joints (n,)"""
        ...
    def fk_to_matrix(self, joints: np.ndarray) -> np.ndarray: ...
    def self_collision(self, joints: np.ndarray) -> bool: ...
    @property
    def dof(self) -> int: ...
```

**Adapters:**

- `PybulletIKSolver(urdf_path)` — 현재 [`PybulletSolver`](../backend/modules/kinematics/solver.py) refactor. 5DOF (OMX) / 6DOF (SO-101) 둘 다 호환 (PyBullet 의 `calculateInverseKinematics` 가 DOF 자동 처리)
- `MujocoIKSolver(mjcf_path)` — Track C ([random_palletizing.md](random_palletizing.md)) 의 학습 env 와 단일 model 일관성. mink (QP-based IK) 또는 numerical IK 직접
- 어느 adapter 사용할지는 robot 별 config 로 결정 (`solver: pybullet | mujoco`)

### 3.2 sag/link_offset Correction (Decorator)

현재 `PybulletSolver` 안에 sag / link_offset 보정이 박혀있음 ([calibration_apply_flow.md](calibration_apply_flow.md)). 이것을 separate layer (Decorator pattern) 로 빼냄:

```python
class CorrectedIKSolver:
    def __init__(self, inner: IKSolver, link_offset, sag_model):
        self._inner = inner
        ...
    def fk(self, joints):
        pos, quat = self._inner.fk(joints)
        return self._sag.apply_forward(pos, quat, joints)
    def ik(self, pos, quat, seed):
        pos_pre, quat_pre = self._sag.apply_inverse(pos, quat)
        return self._inner.ik(pos_pre, quat_pre, seed)
```

이러면:

- inner solver 가 PyBullet 이든 MuJoCo 이든 보정은 한 번만 짜고 양쪽 다 적용
- robot 별로 다른 sag/offset 가져도 같은 decorator 재사용 (생성자에 robot 별 인스턴스 주입)

### 3.3 MotorBackend (Protocol)

so101_6dof_plan §6.1 의 핵심. Dynamixel / Feetech SDK 가 완전히 다른 protocol.

```python
class MotorBackend(Protocol):
    def read_positions(self) -> dict[int, int]: ...   # raw motor units
    def write_positions(self, cmd: dict[int, int]) -> None: ...
    def read_currents(self) -> dict[int, int]: ...    # for contact detection
    def set_torque(self, ids: list[int], enable: bool) -> None: ...
    def configure_pid(self, ids: list[int], pid: PIDConfig) -> None: ...
    ...
```

**Adapters:**

- `DynamixelBackend(port, baud)` — `dynamixel-sdk`, XL430/330, OpenRB-150
- `FeetechBackend(port, baud)` — `scservo_sdk` / `feetech-servo-sdk`, STS3215/3250, Waveshare
- **Mixed 버스 지원** (so101_6dof_plan §6.1) — `FeetechBackend` 안에서 모터 모델 별 (sts3215 vs sts3250) 분기는 motors.yaml 의 `model` 필드로

`backend/modules/motor/` 내부 구조도 같이 정리 — 현재 dynamixel 만 가정한 부분을 backend-agnostic 으로.

### 3.4 CameraCapture (Protocol)

```python
class CameraCapture(Protocol):
    def get_color_jpeg(self) -> bytes: ...
    def get_depth_frame(self) -> DepthFrame: ...
    def get_intrinsic(self) -> Intrinsic: ...
```

**Adapters:**

- `RealSenseCapture` — 현재 [`RealsenseCapture`](../backend/core/realsense_capture.py)
- `MujocoCapture` — sim 내 가상 카메라 (Track C 학습 env 에서 사용)

D405 한 대를 두 robot 이 공유 가능 (어느 EE 에 마운트). 또는 robot 별 각자. 어느 쪽도 interface 는 동일.

### 3.5 MotionExecutor (Protocol)

```python
class MotionExecutor(Protocol):
    def move_j(self, robot_id: str, joints, **opts) -> ServiceResult: ...
    def move_l(self, robot_id: str, pose, **opts) -> ServiceResult: ...
    def move_tcp(self, robot_id: str, target, **opts) -> ServiceResult: ...
    def get_tcp(self, robot_id: str) -> Pose6: ...
    ...
```

**Adapter**: `MultiRobotMotionExecutor` 단일 구현체. robot_id 받으면 내부에서 해당 robot 의 IKSolver / MotorBackend / TrajectoryRunner 디스패치.

또는 robot 마다 별도 MotionNode 인스턴스 + 라우팅 layer — 어느 쪽이 깨끗한지는 11. Open questions 참조.

### 3.6 Coordinator (★ 신규 layer)

OMX 단독에서 존재하지 않던 영역. 두 robot 동시 동작 시:

- **Workspace conflict avoidance** — 두 robot 의 motion plan 사전 collision check (PyBullet 의 multi-robot world 에 둘 다 로드), run-time monitoring (joint state cache 의 robot 별 현재 자세)
- **Synchronization primitives** — `SyncBarrier`, `Lock`, `Channel` — 두 robot 의 phase 동기화 (예: "둘 다 hover pose 도달 후 동시 descend")
- **Task allocation** — 휴리스틱 (어느 박스를 어느 robot 이) → 추후 RL learn 가능 (Track C 확장)
- **Handoff sequencing** — 한 robot 이 객체 들고 → 다른 robot 의 그리퍼 도달 → 둘 다 잡은 상태 sync → 첫 robot release → 두 번째 robot 만 잡은 상태. 각 phase 의 sync + force/timing 조율

Coordinator 가 새 노드인지, MotionExecutor 안의 sub-component 인지, Step DSL 의 step 인지 — 11. Open questions 참조.

---

## 4. Robot identity 모델

### 4.1 robot_id 차원 도입 위치

so101_6dof_plan §6.4 의 "지금 미리 over-generalize 하지 말고, 실물 통합 시점에 자연스러운 분기점에서 robot_id 차원 도입" 룰 그대로. 단 design 은 미리 정리:

| 영역                            | robot_id 차원 도입?         | 도입 시점            |
| ------------------------------- | --------------------------- | -------------------- |
| `IKSolver` (Protocol)           | X (per-robot 인스턴스)      | Phase 1              |
| `MotorBackend` (Protocol)       | X (per-robot 인스턴스)      | Phase 1              |
| `JointStateCache`               | ✓ (dict[robot_id] → state)  | Phase 2 (SO-101 도착) |
| `Coordinates` (Joint/Link/Sag)  | ✓ (robot 별 npz 분리)       | Phase 2              |
| `MotionExecutor.move_*()` API   | ✓ (robot_id 인자)            | Phase 2              |
| Step DSL `Step`                 | ✓ (robot_id 인자 또는 context) | Phase 2          |
| 토픽 prefix                     | ✓ (`<robot_id>/*`)           | Phase 1 (정정 차원)  |
| 캘리브레이션 산출물 디렉토리    | ✓ (`robot/calibration/<id>/`) | Phase 1              |

### 4.2 robot_type vs robot_instance

현재 두 type 각 1대 운용. type=id 로 충분:

- `robot_id = "omx_f"` (type 이름 그대로)
- `robot_id = "so101_6dof"` (type 이름 그대로)

미래에 같은 type 의 robot 이 여러 대 운용되면 그때 instance 차원 (`omx_f_0`, `omx_f_1`) 분리. 지금은 가능성만 인지하고 type=id 로 통합.

### 4.3 robot config (`robot/config/robots.yaml` 신규)

```yaml
robots:
  omx_f:
    type: omx_f
    dof: 5
    urdf: robot/urdf/omx_f/omx_f.urdf
    motor_backend: dynamixel
    motor_config: robot/config/motors_omx_f.yaml
    iksolver: pybullet
    calibration_dir: robot/calibration/omx_f/

  so101_6dof:
    type: so101_6dof
    dof: 6
    urdf: robot/urdf/so101_6dof/so101_6dof.urdf
    motor_backend: feetech
    motor_config: robot/config/motors_so101_6dof.yaml
    iksolver: pybullet     # 또는 mujoco
    calibration_dir: robot/calibration/so101_6dof/

cooperation:
  primary: omx_f           # lead-follow 시나리오의 default lead
  workspace_overlap: ...   # 두 robot 의 reach mask 교집합 정의
  robot_to_robot_extrinsic: robot/calibration/robot_to_robot.npz
```

---

## 5. 디렉토리 구조 변경

### 5.1 robot/ 재구성

```
robot/
├── urdf/
│   ├── omx_f/          (기존 그대로)
│   └── so101_6dof/     (신규 — so101_plan §6.2 의 작업)
├── calibration/
│   ├── omx_f/          (이동: 기존 산출물 다 여기로)
│   │   ├── intrinsic.npz
│   │   ├── hand_eye.npz
│   │   ├── joint_offset.npz
│   │   ├── link_offset.npz
│   │   └── sag_offset.npz
│   ├── so101_6dof/     (신규)
│   │   └── (동일 5종)
│   └── robot_to_robot.npz   (★ 신규 — 7번째 캘 산출물)
├── config/
│   ├── robots.yaml     (★ 신규 — robot list, §4.3)
│   ├── motors_omx_f.yaml      (rename: motors.yaml → motors_omx_f.yaml)
│   └── motors_so101_6dof.yaml  (신규)
└── models/             (기존 그대로 — TSDF mesh 등)
```

### 5.2 backend/modules 정리

```
backend/modules/
├── kinematics/
│   ├── iksolver.py            (★ 신규 — Protocol + decorator)
│   ├── adapters/
│   │   ├── pybullet_solver.py    (현재 solver.py rename + refactor)
│   │   └── mujoco_solver.py      (★ 신규)
│   └── trajectory_runner.py   (robot_id 인자 추가)
├── motor/
│   ├── backend.py             (★ 신규 — Protocol)
│   ├── adapters/
│   │   ├── dynamixel_backend.py
│   │   └── feetech_backend.py    (★ 신규)
│   └── ...
├── coordination/              (★ 신규 폴더)
│   ├── conflict_check.py
│   ├── sync_primitives.py
│   ├── task_allocator.py
│   └── handoff.py
└── ...
```

### 5.3 backend/nodes 정리

현재 단일 robot 가정 노드들 → robot_id 차원:

- `motor_node` — robot_id 별 인스턴스 (configured by `robots.yaml`) 또는 multi-robot 단일 노드 내부 라우팅
- `motion_node` — 동일
- 신규 `coordinator_node` — Coordination layer 호스팅

---

## 6. 토픽 namespace

### 6.1 현재 (혼란)

```
omx/motion/cmd_j          ← "omx" 가 시스템 이름인지 robot 이름인지 모호
omx/system/heartbeat      ← system 이라면서 omx prefix
omx/camera/stream/raw     ← omx 라는 시스템 안의 camera
...
```

`omx` 가 시스템 이름처럼 박혀있지만 사실 robot 종류 1개일 뿐. 이름 박살.

### 6.2 미래 (깨끗한 분리)

```
<robot_id>/motion/cmd_j            ← robot scoped
                                     예: omx_f/motion/cmd_j
                                          so101_6dof/motion/cmd_j
<robot_id>/motor/state/joint
<robot_id>/camera/stream/raw       (eye-in-hand 면 robot scoped)

system/heartbeat                    ← cross-robot 시스템 (★)
system/log                          ← 시스템 전체 로그
system/time                         ← time sync

coord/handoff_state                 ← coordination layer (★ 신규)
coord/workspace_lock
coord/conflict_check
coord/task_allocation

task/tree                           ← task plan (어느 robot 사용은 페이로드 안에)
task/state
task/step_result

viz/palletizer/state                ← 시각화 전용
viz/...
```

### 6.3 마이그레이션 매핑

| 현재                          | 미래                            |
| ----------------------------- | ------------------------------- |
| `omx/motion/cmd_j`            | `omx_f/motion/cmd_j`            |
| `omx/motor/state/joint`       | `omx_f/motor/state/joint`       |
| `omx/camera/stream/raw`       | `omx_f/camera/stream/raw`       |
| `omx/system/heartbeat`        | `system/heartbeat`              |
| `omx/system/log`              | `system/log`                    |
| `omx/task/tree`               | `task/tree`                     |
| `omx/pointcloud/stream`       | `omx_f/pointcloud/stream`       |
| `omx/palletizer/state`        | `viz/palletizer/state`           |

[backend/core/topic_map.py](../backend/core/topic_map.py) 와 [frontend/src/constants/topics.ts](../frontend/src/constants/topics.ts) 두 곳 다 정정 필요 (CLAUDE.md "토픽/서비스 레지스트리 — 두 곳에서 동기화" 룰).

### 6.4 시스템 이름 prefix 박지 말기

repo 이름이 `horibot` 든 무엇이든 토픽 prefix 에 박지 않음. ROS 관례, anti-pattern. 시스템 이름 변경 시 토픽 깨짐 방지.

---

## 7. Typed payload schema (★ 새 영역 — 깊은 논의 별도)

### 7.1 동기

현재 토픽 페이로드는 모두 JSON dict (CLAUDE.md "토픽 페이로드는 보통 JSON"). 서비스 봉투도 `{success, message, data}` 의 `data` 가 free-form dict. 즉:

- 보낼 때 / 받을 때 양쪽이 정확한 dict 모양 알고 있어야 함 (불일치 시 runtime error)
- Frontend ([topics.ts](../frontend/src/constants/topics.ts)) 와 Backend ([topic_map.py](../backend/core/topic_map.py)) 가 **토픽 이름** 은 두 곳에 박혀있지만 **페이로드 schema** 는 어디에도 명시적 정의 없음

Multi-robot 시나리오에서 이게 빠르게 한계:

- 토픽 수 폭발 (robot 별 + coordination)
- `robot_id` 필드가 모든 페이로드에 추가 — 일관성 강제 필요
- Coordinator 페이로드 (예: `coord/handoff_state`) 가 복잡한 nested 구조
- Step DSL 의 typed Slot ([step_dsl.md](step_dsl.md)) 이 typed payload 와 합쳐지면 end-to-end 타입 안정성

### 7.2 옵션 후보

- **Pydantic models** — Python first-class, JSON schema 자동 generation. Backend 만족
- **dataclass + JSON schema** — Step DSL 의 typed Slot 과 같은 톤. 가벼움
- **Protobuf** — `.proto` 정의 → 다국어 codegen (Python + TS). binary 토픽 (현재 pointcloud) 과 자연스러움. 산업 표준
- **TypedDict** — 가벼움, 정적 타입 체크만, runtime 검증 X

### 7.3 Frontend schema 공유

- **두 곳 동기화** — 현재 토픽 이름과 같은 패턴. 단순하지만 drift 위험
- **Single source of truth + codegen** — JSON schema 또는 `.proto` 한 군데 → Python + TS 자동 생성. 안전하지만 빌드 인프라 신규
- **Hybrid** — 일부 core 토픽만 codegen, 나머지는 free-form

### 7.4 도입 시점

- Phase 1 (Interface 도입) 시점에 같이 하기엔 영역 너무 큼
- Phase 2 (SO-101) 시점이 자연스러움 — 어차피 새 robot 의 토픽 추가하면서 schema 일관성 가치 큼
- 또는 별도 phase

### 7.5 별도 design 문서 필요

이 영역은 본 문서 scope 보다 큼. 별도 design 문서 (`docs/typed_payload_schema.md` 또는 비슷) 로 깊이 논의 필요. 본 문서에서는 **필요성 인지 + multi-robot architecture 와의 의존성** 만 표기.

---

## 8. Step DSL multi-robot 확장

### 8.1 robot_id 인자 추가

현재 [step_dsl.md](step_dsl.md) 의 primitive (MoveJByName / MoveTCP / Gripper / GroundedDetect / GraspPolicy / PlacePolicy 등) 는 "the robot" 가정. multi-robot 으로 가면:

```python
# 옵션 (a): step 에 robot 인자 명시
MoveTCP(robot="omx_f", target=pos, offset=Position3(0, 0, 0.06))

# 옵션 (b): context 에 robot scope, step 은 인자 없음
with RobotScope("omx_f"):
    MoveTCP(target=pos, offset=Position3(0, 0, 0.06))

# 옵션 (c): hybrid — context 가 default, step 에서 override 가능
```

→ Open question (11.). 옵션 (a) 가 가장 explicit + Slot reference 와 일관 (각 step 이 self-contained), (b) 는 nested task 의 robot context 자동 propagation. (c) 가 절충.

### 8.2 Coordinator steps (★ 신규)

두 robot 의 동시 / 순차 / 동기 제어를 위한 step primitive:

- **`ParallelExec([step_a, step_b])`** — 두 robot 의 step 동시 실행. 둘 다 끝나야 다음. PyBullet 사전 collision check 통과 필수
- **`Handoff(from_robot, to_robot, object_slot)`** — 한 robot 이 잡은 객체를 다른 robot 으로 이전. 내부적으로 (a) to_robot 의 approach pose, (b) sync barrier (둘 다 잡음), (c) from_robot release, (d) from_robot retreat 의 sub-step 으로 분해
- **`SyncBarrier([slot_a, slot_b])`** — 두 robot 이 각자의 phase 도달 후 동시 다음 step
- **`Lock(workspace_region)`** — 특정 workspace region 을 한 robot 만 점유. context manager 처럼 동작

### 8.3 typed Slot 의 robot-aware-ness

기존 `Slot[Position3]` 같은 typed Slot ([step_dsl.md](step_dsl.md) §6) 에 robot frame 정보 추가:

```python
Slot[Position3]                # frame 안 명시 (현재) — base frame 가정
Slot[Position3, "omx_f.base"]  # explicit frame (미래)
```

또는 별도 wrapper:

```python
@dataclass(frozen=True)
class FramedPosition3:
    pos: Position3
    frame: str  # "omx_f.base" | "so101_6dof.base" | "world"
```

Cooperation 시 두 robot 의 frame 변환 (robot-to-robot extrinsic 사용) 필요한 데 이 정보가 schema 에 박혀야 자동 변환 가능.

---

## 9. 캘리브레이션 산출물

### 9.1 Robot 별 5종 분리 (so101_plan §6.6 그대로)

```
robot/calibration/
├── omx_f/
│   ├── intrinsic.npz
│   ├── hand_eye.npz
│   ├── joint_offset.npz
│   ├── link_offset.npz
│   └── sag_offset.npz
├── so101_6dof/
│   └── (동일 5종)
└── robot_to_robot.npz   ← 신규 (§9.2)
```

각 robot 별 [calibration_apply_flow.md](calibration_apply_flow.md) 의 4종 적용 메커니즘 그대로. URDF patch (link_offset) / sag 양방향 적용 / joint_offset / hand_eye / intrinsic — adapter 단위로 분리.

### 9.2 Robot-to-Robot extrinsic (★ 신규 7번째 캘)

**역할**: 두 robot 의 base frame 이 world frame 에서 서로 어디 있는지 변환 행렬. cooperative manipulation 의 필수 prerequisite.

**왜 필수**:

- OMX_F base frame 에서 객체 위치 = (x, y, z) — SO-101 의 base frame 에서는 같은 객체가 (x', y', z') 로 보임
- handoff / bimanual 시 두 robot 의 좌표계를 정확히 변환할 수 있어야 협업 가능
- σ 가 클수록 두 robot 그리퍼 사이 misalignment → handoff 실패 / 객체 떨어짐 / 충돌

**캘 방법 후보**:

1. **Shared marker** — 두 robot 의 EE 가 각자 같은 marker (예: ChArUco / AprilTag) 를 여러 각도에서 관찰 → 두 hand_eye 결과 비교로 robot-to-robot extrinsic 도출
2. **Cross-touch** — 한 robot 이 다른 robot 의 EE 의 정확한 지점 (마커 부착) 을 multiple pose 에서 touch → ICP / least-squares 로 변환 추정. 정확하지만 충돌 risk
3. **외부 fixed camera** — 두 robot 의 EE 가 모두 보이는 외부 camera 한 대 추가 → 두 robot 의 hand_eye 동시 풀이 + 변환 추출

**산출물 형식**: `robot_to_robot.npz` 에 4x4 변환 행렬 (omx_f base → so101_6dof base 또는 둘 다 world base 로 변환).

**적용**: Coordinator layer 가 두 robot 간 frame 변환 필요할 때 자동 적용. Step DSL 의 `FramedPosition3` 도 이 변환 사용.

**정확도 목표**: hand_eye 의 σ_t 7.94mm 보다 동등 또는 ↑. 두 robot σ 누적 시 cooperative grasp 의 정확도 천장이 됨.

### 9.3 캘 절차 신규 — `calibration_robot_to_robot.md`

별도 design 문서로 분리 가치. 본 문서에서는 필요성 + 후보 방법만.

---

## 10. Coordination layer 세부

### 10.1 공조 패턴 4종

so101_plan §1 의 listing 그대로 + 일반 명명:

| 패턴             | 정의                                      | 예시                                |
| ---------------- | ----------------------------------------- | ----------------------------------- |
| **Bimanual**     | 두 robot 이 같은 객체 동시 잡고 함께 조작 | 큰 박스 들기, 양손 어셈블리         |
| **Handoff**      | 한 robot 의 객체를 다른 robot 에게 전달   | 부품 전달, workspace 간 이동        |
| **Parallel**    | 독립 작업, workspace 만 공유               | 한 robot pick / 다른 robot place    |
| **Lead-follow** | 한 robot 주도, 다른 robot 보조             | 한 손 잡고 다른 손이 조작           |

각 패턴별 다른 Coordinator step / sync primitive 필요.

### 10.2 Workspace conflict avoidance

**사전 (plan-time)**:

- 두 robot 의 motion plan 을 PyBullet 의 multi-robot world (두 URDF + robot-to-robot extrinsic) 에 동시 로드 → collision check
- collision 발견 시 plan 재생성 또는 시간 offset

**Run-time**:

- JointStateCache 의 robot 별 현재 자세를 100Hz 로 모니터링
- 두 robot 의 EE distance / link bounding box 충돌 검사
- 임계치 이하 시 즉시 정지 (Trajectory Runner 의 abort)

**Workspace zone 분리**:

- `robots.yaml` 의 `cooperation.workspace_overlap` 정의
- non-overlap region 에서는 conflict check skip (성능 ↑)
- overlap region 에서만 정밀 check

### 10.3 Synchronization primitives

```python
# Coordinator API 예시
async with workspace_lock("zone_a"):
    await move_tcp("omx_f", target_a)

barrier = SyncBarrier(robots=["omx_f", "so101_6dof"])
await barrier.arrive_and_wait("omx_f")   # so101 이 도착할 때까지 wait
await barrier.arrive_and_wait("so101_6dof")  # 둘 다 도착 후 다음 phase
```

내부적으로 Zenoh 토픽 (`coord/sync_barrier`, `coord/workspace_lock`) 으로 두 robot 의 host 간 동기 (분산 환경: PC + 모터 Pi 1 + 모터 Pi 2 일 경우).

### 10.4 Task allocation

**휴리스틱** (Phase 3 시작점):

- "더 가까운 robot 이 pick" (reach 거리 기반)
- "더 free 한 robot 이 다음 task" (현재 busy 상태 기반)
- "fixed assignment" (config 의 primary/secondary)

**RL learn 후보** (Phase 4):

- random_palletizing 의 Track C 확장 — packing 정책 학습 + 어느 robot 이 어느 박스 결정 동시 학습
- multi-agent RL (independent / centralized critic)

### 10.5 Coordinator 구현 위치 — Open

- (a) **신규 노드** `coordinator_node` — PC 에서 실행, Zenoh 로 두 motion node 와 통신. 분산 깨끗
- (b) **MotionExecutor 내부** — 단일 노드 안의 sub-component. 통신 overhead ↓
- (c) **Step DSL 내부** — Coordinator step 이 Zenoh 토픽 publish 로 직접 sync, 별도 노드 X

→ 11. Open questions

---

## 11. Frontend 변경

### 11.1 URDF 2개 동시 렌더링

[Workspace3D](../frontend/src/pages/Workspace3D.tsx) 의 `urdf-loader` 를 두 robot 동시 호스팅:

```tsx
<URDFLoader src="/robot/urdf/omx_f/omx_f.urdf"
            joints={omxJoints}
            basePose={omxBasePose} />
<URDFLoader src="/robot/urdf/so101_6dof/so101_6dof.urdf"
            joints={so101Joints}
            basePose={so101BasePose} />
```

`basePose` 는 robot-to-robot extrinsic 으로부터 도출. 두 robot 이 world frame 에서 정확한 상대 위치로 렌더링.

### 11.2 robot 별 panel

현재 단일 robot 가정 패널 ([panelComponents.ts](../frontend/src/components/workspace3d/dockview/panelComponents.ts)) 들:

- Motion panel — robot 별 탭 또는 robot 별 패널 2개
- Calibration panel — robot 별 탭
- Joint state panel — robot 별 view

`dockview` 의 다중 panel 자연스럽게 확장 가능.

### 11.3 Coordination 시각화 (★ 신규)

- **Workspace overlap region** — 두 robot 의 reach mask 교집합을 3D scene 에 reach-zone overlay
- **Handoff zone** — 두 robot 의 그리퍼가 만날 수 있는 영역 highlight
- **Conflict warning** — 두 robot 의 EE 거리 임계 이하 시 색 변경
- **Active sync barrier** — 어느 robot 이 wait 중인지 indicator

### 11.4 Stores 분리

Zustand store ([frontend/src/store/](../frontend/src/store/)) 의 robotStore / motionStore 를 robot 별 sub-state 로:

```ts
robotStore: {
  robots: {
    "omx_f": { joints, basePose, calibration, ... },
    "so101_6dof": { ... }
  },
  cooperation: { handoffState, lockedZones, ... }
}
```

---

## 12. 마이그레이션 phase plan

### Phase 0 — Design 정리 (현재)

이 문서 + (선택) 별도 `typed_payload_schema.md` / `calibration_robot_to_robot.md` design 문서 작성. 구현 X.

### Phase 1 — Interface 도입 (OMX 만 implement, 동작 변화 0)

**목표**: abstraction layer 도입하되 동작 변화 0 (regression test 통과). OMX 만 작동, SO-101 은 stub.

작업:

1. ~~**Repo 이름 변경** (§2.4)~~ ✅ 선행 완료 (commit `1114524`)
2. **IKSolver Protocol** 정의 ([iksolver.py](../backend/modules/kinematics/iksolver.py)) + `PybulletSolver` → `PybulletIKSolver` rename / interface 만족 refactor
3. **CorrectedIKSolver Decorator** — sag / link_offset 보정 별도 layer 분리
4. **MotorBackend Protocol** 정의 + `DynamixelBackend` adapter (현재 코드 wrap)
5. **CameraCapture Protocol** + `RealSenseCapture` adapter
6. **토픽 namespace 정정** — `omx/*` → `omx_f/*` + `system/*` 분리 + `viz/*` 분리. [topic_map.py](../backend/core/topic_map.py) + [topics.ts](../frontend/src/constants/topics.ts) 둘 다 갱신
7. **디렉토리 재구성** (§5) — `robot/calibration/omx_f/` 로 이동 + `robot/config/robots.yaml` 신규
8. **호출처 갱신** — MotionNode / Detector / TrajectoryRunner 모두 Protocol 의존으로
9. **Regression test** — 동작 변화 0 검증

이 phase 끝나면: 코드는 깨끗 + interface 깔림 + SO-101 추가 시점에 매끄러운 분기점 마련

### Phase 2 — SO-101 도착 → 두 번째 adapter

so101_6dof_plan §6, §7 의 작업 그대로 + interface 위에서:

1. **URDF 배치** (`robot/urdf/so101_6dof/`)
2. **FeetechBackend** adapter 신규
3. **PybulletIKSolver(so101_6dof.urdf)** 인스턴스 생성 (5DOF/6DOF 둘 다 호환)
4. **JointStateCache / Coordinates 의 robot_id 차원 도입**
5. **Step DSL 의 robot_id 인자** 추가 (옵션 a/b/c §8.1 결정)
6. **캘리브레이션 5종** SO-101 용 새로 산출 + `robot/calibration/so101_6dof/`
7. **모터 Pi 2 (SO-101 전용)** 분산 토폴로지 추가 — 또는 1 모터 Pi 가 두 controller 동시 처리
8. **두 robot 단독 동작 검증** — cooperative 없이 각자 motion 정상

### Phase 3 — Coordinator 도입

1. **Robot-to-Robot extrinsic 캘** (§9.2)
2. **Coordinator node / sub-component** (§10.5 결정)
3. **Coordinator steps** (`ParallelExec` / `Handoff` / `SyncBarrier`) Step DSL 통합
4. **Workspace conflict avoidance** (사전 + run-time)
5. **첫 공조 task** — 가장 간단한 패턴부터 (parallel 또는 handoff). bimanual 은 후순위 (force coordination 어려움)
6. **Frontend URDF 2개 + Coordination 시각화**

### Phase 4 — 공조 task 확장

1. Bimanual / lead-follow 패턴 task
2. Track C (RL palletizing) multi-robot 확장 — task allocation 학습
3. Typed payload schema 도입 (§7.4 — phase 별도일 수도)
4. 추가 design 문서 / 개선

---

## 13. Open questions (결정 대기)

본 문서 토대로 구현 시작 전 / 진행 중 결정 필요한 항목:

1. ~~**레포 이름 최종 선택**~~ — ✅ **결정: `horibot`** (§2.2 참조)
2. **MotionExecutor 구조** (§3.5) — 단일 multi-robot executor vs robot 별 노드 + 라우팅
3. **Coordinator 구현 위치** (§10.5) — 신규 노드 / MotionExecutor 내부 / Step DSL 내부
4. **Step DSL 의 robot_id 표기** (§8.1) — 옵션 (a) explicit / (b) context / (c) hybrid
5. **공조 첫 task 선택** (Phase 3) — parallel pick&place / handoff / bimanual 중
6. **Robot-to-Robot 캘 방법** (§9.2) — shared marker / cross-touch / 외부 fixed camera
7. **Typed payload schema 옵션** (§7.2) — Pydantic / dataclass / Protobuf / TypedDict
8. **Schema codegen 인프라** (§7.3) — 두 곳 동기화 / single source codegen / hybrid
9. **MuJoCo IK adapter 도입 시점** — Phase 1 안에 / Phase 2 / Phase 3 / Track C 작동 후
10. **분산 토폴로지** — 모터 Pi 2대 vs 1대로 통합 (USB 대역폭 / latency tradeoff)
11. **Frontend dock view layout** (§11) — 두 robot panel 좌우 배치 / 탭 / 자유 dock
12. **Cooperation config** (§4.3 `robots.yaml`) 의 workspace_overlap 표현 — bounding box / polygon / 격자

---

## 14. 의존성 / 관련 문서

**Prerequisite 문서**:

- [so101_6dof_plan.md](so101_6dof_plan.md) — 하드웨어 조립 + 모터 SDK 추상화 (§6) 의 본 문서와의 의존성. Phase 2 의 직접 입력
- [step_dsl.md](step_dsl.md) — Step DSL 토대 — §8 의 확장 base
- [calibration_apply_flow.md](calibration_apply_flow.md) — 4종 캘 적용 메커니즘 — §9 의 robot 별 분리 대상
- [hand_eye_extended_ba.md](hand_eye_extended_ba.md) — 확장 BA + sag 모델 — SO-101 캘에 그대로 적용 가능

**별도 design 문서로 분리 후보** (이 문서가 너무 커지면):

- `docs/kinematics_solver_interface.md` — §3.1-3.2 의 IKSolver / CorrectedIKSolver detail (본 문서로 흡수됨, 별도 분리 시 cross-ref)
- `docs/typed_payload_schema.md` — §7 의 깊은 design
- `docs/calibration_robot_to_robot.md` — §9.2 의 캘 절차 detail
- `docs/coordination_layer.md` — §10 의 sync primitives / task allocation detail

**Task 별 적용 예시**:

- [random_palletizing.md](random_palletizing.md) — 본 architecture 위에서 multi-robot palletizing 어떻게 변하는지 — Phase 3-4 에서 same task 의 multi-robot 확장 추가

---

## 15. 다음 design 영역 (Backlog)

본 문서 작성 과정에서 brainstorm 한 후속 design 후보. 카테고리별 정리. SO-101 도착 / 본 architecture 의 Phase 진행에 따라 우선순위 변함.

### A. 자연스러운 후속 design 문서 (이 architecture 의 직계 후속)

- **Typed payload schema** — §7 의 깊은 논의. dict → typed 전환 영향이 코드 전반. Pydantic vs dataclass vs Protobuf 선택 + Frontend codegen 인프라. **Phase 2 시점 needed**
- **Robot-to-Robot extrinsic 캘 절차** — §9.2 의 후보 3개 (shared marker / cross-touch / 외부 fixed camera) 의 실제 절차 / 정확도 검증 / 산출물 schema. **SO-101 도착 즉시 needed**
- **Coordinator layer detail** — §10 의 sync primitives / handoff sequencing / workspace conflict 의 actual 구현 detail. **Phase 3 시작 전 needed**

### B. Critical 공백 (현재 design 없음)

- **Error handling / safety / E-stop** — 단일 robot 에서도 명시적 design 없음. multi-robot 가면 두 robot 충돌 직전 정지 / fail-safe / 한쪽 fail 시 다른쪽 처리 / E-stop 통합. **critical**
- **Logging / observability 통합** — Track C ([random_palletizing.md](random_palletizing.md)) 의 `cycle_log` 만 짧게. 시스템 전반 logging schema / structured log / 검색·분석 도구 (Loki / Grafana / 단순 파일) / multi-robot trace correlation
- **CI / Testing 인프라** — Mock adapter / regression baseline / integration test / 캘 정확도 회귀 측정. Phase 1 의 "동작 변화 0 검증" 의 발판

### C. 다른 갈래 (이 architecture 와 직교)

- **Imitation learning 데이터 수집 인프라** — [ideas.md](ideas.md) 의 Kinesthetic teaching. Seeed 듀얼 키트로 SO-101 leader teleop 가면 데이터 수집 가능. ACT / Diffusion Policy / VLA prerequisite
- **외부 fixed camera 검토** — cooperative manipulation 의 visibility (두 robot 의 EE 모두 보이는 view 필요할 수도) + robot-to-robot 캘 옵션
- **Multi-camera fusion** — 두 D405 의 포인트클라우드 합치기. 기존 TSDF pipeline ([tsdf_pipeline.md](tsdf_pipeline.md)) 확장. workspace 사각지대 해소

### D. 장기 / 큰 변화

- **LLM orchestrator multi-robot 확장** — [ideas.md](ideas.md) 의 ★ 유력 항목이 multi-robot 가면 step 에 robot 선택까지 LLM 이 결정. Coordinator 와 통합
- **Multi-agent RL** — Track C 의 multi-robot 확장. task allocation 학습 (independent / centralized critic 등)
- **새 task 도메인** — 양손 어셈블리 / 페그-홀 / 책상 정리 / 와이어 라우팅 / 부드러운 객체. 현재 box manipulation 위주
- **Tactile / Force sensing** — vision 만 가지고는 cooperative grasp force 조율 / handoff force sync 한계. force sensor / current sensing 통합

### E. Process

- **Documentation 정리** — docs/ 가 점점 늘어남 (~10개+). topic 별 sub-folder / index 페이지 / cross-link 정리 검토 시점
- **Open-source 화** — README / contributing / examples. 학습 platform 으로 공개 시 reproducibility (URDF / 캘 / 모델 / config) 챙기기

### 우선순위 (작성 시점)

| 순위 | 영역                                                       | trigger                                     |
| ---- | ---------------------------------------------------------- | ------------------------------------------- |
| 1    | A 의 3개 (Typed payload / Robot-to-Robot 캘 / Coordinator) | 이 design phase 의 자연스러운 마무리        |
| 2    | B 의 3개 (Error handling / Logging / CI)                   | critical 공백 — multi-robot 가기 전 정리    |
| 3    | C/D/E                                                       | SO-101 도착 후 / Phase 2-4 진행에 따라 ↑    |
