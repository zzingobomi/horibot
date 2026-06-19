# Horibot

OMX_F + SO-101 6DOF 로봇팔 제어 스택.

- **Backend** (Python 3.11, uv) — Dynamixel/Feetech 서보 제어, D405 RGBD 캡처, YOLO 디텍션, Hand-Eye 캘리브레이션, Ruckig trajectory, PyBullet FK/IK, Zenoh pub/sub
- **Frontend** (React + Vite + R3F) — teleop / 캘리브레이션 / 3D 디지털 트윈 워크스페이스
- **Bridge** — FastAPI + WebSocket이 브라우저와 Zenoh를 연결

## 빠른 시작

### 단일 머신 (개발용)

```powershell
cd backend
uv sync
uv run python main.py              # host_<hostname>.yaml 자동 매칭, fallback = host_dev.yaml
```

```powershell
cd frontend
pnpm install
pnpm dev                           # http://localhost:5173
```

브릿지는 기본 `ws://localhost:8000/ws` / `http://localhost:8000`.

### 분산 환경 (PC + 모터 Pi + 카메라 Pi)

같은 코드베이스를 세 머신이 공유하고, 각자의 host config로 어떤 노드를 띄울지 결정. Zenoh가 LAN 안에서 자동 peer discovery — 노드는 자기가 어디서 도는지 알 필요 없음.

| 머신 | 노드 | 책임 |
|---|---|---|
| PC | detector / task / scene3d / reconstruction / storage / calibration / bridge / gamepad | YOLO, Open3D, TSDF, storage gateway, 브릿지 |
| 모터 Pi (192.168.0.101) | motor / motion | Dynamixel + Ruckig + IK (제어 루프 로컬화) |
| 카메라 Pi (192.168.0.102) | camera | D405 캡처 + JPEG + 압축 depth 발행 |

```powershell
# PC
cd backend
uv sync                                    # default-groups (dev + all)
uv run --no-sync python main.py --host pc
```

```bash
# 모터 Pi
cd backend
uv sync --no-default-groups --group pi-motor
uv run --no-sync python main.py --host pi_motor
```

```bash
# 카메라 Pi (pyrealsense2는 사전 소스 빌드 후 별도 install — docs/pyrealsense2-build-guide.md)
cd backend
uv sync --no-default-groups --group pi-camera --no-install-package pyrealsense2
uv pip install ~/pyrealsense2-2.55.1-cp311-cp311-linux_aarch64.whl
uv run --no-sync python main.py --host pi_camera
```

분산 아키텍처 자세히는 [CLAUDE.md § 아키텍처](CLAUDE.md) / [docs/distributed_topology.md](docs/distributed_topology.md).

## 문서

| 문서                                                                 | 내용                                                            |
| -------------------------------------------------------------------- | --------------------------------------------------------------- |
| [CLAUDE.md](CLAUDE.md)                                               | 프로젝트 SSOT — 아키텍처 / 토폴로지 / 노드 패턴 / 규약          |
| [docs/](docs/)                                                       | 주제별 상세 문서 (hardware / calibration / motion / storage 등) |
| [docs/storage_layer.md](docs/storage_layer.md)                       | 영속성 layer (RDB + ObjectStore) 설계 + Alembic 운영            |
| [docs/multi_robot_architecture.md](docs/multi_robot_architecture.md) | Multi-robot 일반화 (OMX_F + SO-101)                             |
| [docs/calibration_workflow.md](docs/calibration_workflow.md)         | 캘리브레이션 절차                                               |
| [docs/roadmap.md](docs/roadmap.md)                                   | 진행 중 / 예정 작업                                             |

> README 대규모 리팩토링 예정. 현재는 anchor 수준.

---

# DB schema 변경 (Alembic) — daily cheat sheet

자세한 절차 / 정책 / 트러블슈팅은 [docs/storage_layer.md § 8.4](docs/storage_layer.md). 본 섹션은 일상 workflow 빠른 참조용.

## Mental model

```
ORM 모델 (backend/modules/<entity>/orm.py)   ← schema의 SSOT (declarative)
        ↓
alembic revision --autogenerate              ← 직전 head와 metadata diff
        ↓
revision file (versions/<date>-<hash>.py)    ← 변경 의도 박제 (이후 불변)
        ↓
git commit                                   ← 팀/머신 sync
        ↓
부팅 시 _ensure_schema (자동)                ← head까지 차분 upgrade
```

핵심: ORM = "있어야 할 모습", revision = "여기서 저기로 어떻게 가는지", DB = "지금 어디". 셋 사이 동기화를 Alembic이 관리.

## Case A — 기존 entity에 column 추가 / type 변경 / index 추가

```python
# 1. ORM 모델 수정 — backend/modules/<entity>/orm.py
class CalibrationRunOrm(Base):
    # ... 기존 ...
    quality_score: Mapped[float | None] = mapped_column(Float, nullable=True)
```

```powershell
# 2. autogenerate
cd backend
uv run alembic revision --autogenerate -m "add quality_score to calibration_run"
```

```
# 3. 생성된 revision 검토 — backend/alembic/versions/<date>-<hash>_<slug>.py
#    - op.add_column 잘 잡혔나
#    - downgrade 잘 만들어졌나

# 4. git commit (orm.py + revision file 같이)
# 5. 다음 backend 부팅 시 _ensure_schema가 자동 upgrade head — 별도 명령 X
```

## Case B — 새 entity (테이블 묶음) 추가

```python
# 1. backend/modules/<new_entity>/orm.py 새로 생성. Base 상속 + Mapped[...] columns
class TaskRunOrm(Base):
    __tablename__ = "task_runs"
    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    # ... columns ...

# 2. ★ backend/alembic/env.py에 import 한 줄 추가
import modules.task_runs.orm  # noqa: E402, F401
#    ↑ 빼먹으면 Base.metadata가 새 테이블을 모름 → autogenerate 빈 결과

# 3. (필요 시) persistence_models.py + ORM↔Record mapper (boundary)
# 4. autogenerate + 검토 + commit + 부팅
```

## Case C — 데이터 backfill (DDL 외 값 채우기)

autogenerate는 _DDL만_ 잡음. data migration은 수동:

```python
# revision file의 def upgrade() 끝에 수동 추가
def upgrade() -> None:
    # ... auto-generated DDL ...
    op.add_column('calibration_runs', sa.Column('quality_score', sa.Float(), nullable=True))
    op.execute("UPDATE calibration_runs SET quality_score = 0.0 WHERE quality_score IS NULL")
```

복잡한 backfill (join / row별 처리):

```python
from sqlalchemy.orm import Session
def upgrade() -> None:
    bind = op.get_bind()
    session = Session(bind=bind)
    for row in session.execute(text("SELECT ...")).all():
        # ... 처리 ...
    session.commit()
```

## Case D — column / table rename (위험)

autogenerate는 `drop_column` + `add_column`으로 잡음 → **data loss**. 직접 수정:

```python
def upgrade() -> None:
    with op.batch_alter_table('calibration_runs') as batch_op:
        batch_op.alter_column('old_name', new_column_name='new_name')
```

## 검증 / 디버깅

```powershell
cd backend
uv run alembic current             # DB가 현재 어느 revision
uv run alembic history             # 전체 revision history
uv run alembic upgrade head        # 부팅 안 거치고 수동 적용
uv run alembic upgrade +1          # 다음 revision 한 단계
uv run alembic downgrade -1        # 직전 revision으로 rollback
uv run alembic downgrade <hash>    # 특정 revision
```

## 자주 만나는 trap

| 증상                                           | 원인                                                    | 해결                                                            |
| ---------------------------------------------- | ------------------------------------------------------- | --------------------------------------------------------------- |
| autogenerate가 빈 revision emit                | 새 entity의 env.py import 누락 (Case B ★)               | env.py에 `import modules.<new>.orm` 한 줄                       |
| `UnicodeDecodeError: 'cp949'`                  | alembic.ini에 한국어 박힘 (Windows configparser locale) | alembic.ini는 ASCII-only 유지                                   |
| autogenerate가 매번 같은 변경 detect           | server default mismatch 등 false positive               | revision 손으로 비우거나 ORM의 `server_default` 명시            |
| `Can't locate revision identified by '<hash>'` | revision file git pull 안 됨 / 누군가 삭제              | `alembic/versions/` sync. **revision file 임의로 지우지 말 것** |
| 동시에 두 PR에서 schema 변경 → branch          | base head 동시 점유                                     | `alembic merge <h1> <h2> -m "merge"` 또는 한 PR rebase          |

## 정책

- Revision file은 git tracked — 모든 머신이 동일 history. 임의 삭제 X
- `alembic_version` 테이블 직접 손대지 말 것 (Alembic이 관리)
- `backend/storage/horibot.db`는 git tracked (현재 컨벤션) — 다른 PC가 같은 캘 데이터로 테스트. schema가 바뀌면 옛 DB도 자동 차분 upgrade
- Squash 안 함 — revision history는 그대로 쌓음
