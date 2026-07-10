# Horibot

Multi-robot 팔 제어 스택 — SO-101 6DOF (Feetech + RealSense D405) + OMX_F (Dynamixel + USB 웹캠).

- **Backend** (Python 3.11, uv) — module 기반 framework: Zenoh + msgpack wire, motor/motion(PyBullet IK)/camera/calibration/scan(Open3D TSDF)/detector(GDINO·SAM2)/llm/task/bridge. 같은 코드가 어디 배치되든 그대로 동작 — 분산은 deployment yaml 만 다름.
- **Frontend** (React + Vite + react-three-fiber) — 3D 워크스페이스 + dockview 플로팅 패널 (teleop / 캘리브레이션 / scan / waypoint / task).
- **Bridge** — backend 의 한 module (FastAPI). 브라우저 WS ↔ Zenoh 릴레이 + `/contract.json` 타입 생성 소스.

작업 가이드/규약/문서 인덱스 = [CLAUDE.md](CLAUDE.md). 아키텍처 SSOT = [docs/backend.md](docs/backend.md) + [docs/frontend.md](docs/frontend.md).

## 빠른 시작

### 단일 머신 (하드웨어 없이 — mock)

```powershell
cd backend
uv sync
uv run --no-sync python -m apps.main --host mock   # 전 모듈 + mock driver, bridge :8000
```

```powershell
cd frontend
pnpm install
pnpm dev                                           # http://localhost:5173
```

브릿지 기본 = `ws://localhost:8000/ws` / `http://localhost:8000`. 개발 콘솔 = `GET /dev`.

### 분산 (PC + Pi 3대)

머신마다 [backend/config/deployments/](backend/config/deployments/) 의 yaml 하나로 어떤 module 을 띄울지 결정. Zenoh peer multicast 로 같은 LAN 자동 발견.

| host | 머신 | modules | 책임 |
|---|---|---|---|
| `pc` | PC | camera_decoded / calibration / scene3d / scan / waypoint / detector / llm / task / pick_and_place / bridge | 무거운 연산 (GDINO, Open3D) + 브릿지 + DB |
| `pi_hori1` | Pi | motor / motion (so101_6dof_0) | Feetech + IK — 제어 루프 로컬화 |
| `pi_hori2` | Pi | camera (so101_6dof_0) | D405 캡처 + JPEG + zstd depth |
| `pi_hori3` | Pi | motor / motion / camera (omx_f_0) | Dynamixel + USB 웹캠 |

```powershell
# PC
cd backend
uv sync
uv run --no-sync python -m apps.main --host pc
```

```bash
# pi_hori1 / pi_hori3
cd backend
uv sync --no-default-groups --group pi-hori1     # 또는 pi-hori3
uv run --no-sync python -m apps.main --host pi_hori1

# pi_hori2 (pyrealsense2 는 사전 소스 빌드 — docs/hardware.md)
uv sync --no-default-groups --group pi-hori2 --no-install-package pyrealsense2
uv pip install ~/pyrealsense2-*.whl
uv run --no-sync python -m apps.main --host pi_hori2
```

토폴로지/HW 상세 = [docs/hardware.md](docs/hardware.md).

## 검증

```powershell
cd backend
uv run ruff check . ; uv run pyright
uv run --no-sync pytest -m "not sim" -q    # fast loop
uv run --no-sync pytest -q                 # full (~90s)

cd frontend
pnpm lint ; pnpm vitest run
pnpm test:e2e                              # Playwright headed — mock backend + dev 서버 선행
```

## 문서

| 문서 | 내용 |
| --- | --- |
| [CLAUDE.md](CLAUDE.md) | 작업 가이드 — 명령어 / 구조 지도 / 규약 / 문서 인덱스 / design decision |
| [docs/backend.md](docs/backend.md) | backend 아키텍처 SSOT + **진행 status/세션 handoff (문서 상단)** |
| [docs/frontend.md](docs/frontend.md) | frontend 아키텍처 SSOT (hooks / 패널 / 씬 소유권) |
| [docs/](docs/) | 도메인별 8편 — backend / frontend / hardware / calibration / task / motion / perception / dev_reference (결정·진단 앵커 원문 통합) |

---

# DB schema 변경 (Alembic) — daily cheat sheet

상세 스키마는 [docs/dev_reference.md](docs/dev_reference.md). 본 섹션은 일상 workflow 빠른 참조용.

## Mental model

```
ORM 모델 (backend/modules/<module>/persistence/orm.py)   ← schema의 SSOT (declarative)
        ↓
alembic revision --autogenerate                          ← 직전 head와 metadata diff
        ↓
revision file (alembic/versions/<hash>_<slug>.py)        ← 변경 의도 박제 (이후 불변)
        ↓
git commit                                               ← 팀/머신 sync
        ↓
부팅 시 run_migrations (자동)                            ← head까지 차분 upgrade
```

핵심: ORM = "있어야 할 모습", revision = "여기서 저기로 어떻게 가는지", DB = "지금 어디". 셋 사이 동기화를 Alembic이 관리.

## Case A — 기존 entity에 column 추가 / type 변경 / index 추가

```python
# 1. ORM 모델 수정 — backend/modules/<module>/persistence/orm.py
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
# 3. 생성된 revision 검토 — backend/alembic/versions/<hash>_<slug>.py
#    - op.add_column 잘 잡혔나
#    - downgrade 잘 만들어졌나

# 4. git commit (orm.py + revision file 같이)
# 5. 다음 backend 부팅 시 자동 upgrade head — 별도 명령 X
```

## Case B — 새 entity (테이블 묶음) 추가

```python
# 1. backend/modules/<module>/persistence/orm.py 에 Base 상속 + Mapped[...] columns
class TaskRunOrm(Base):
    __tablename__ = "task_runs"
    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    # ... columns ...

# 2. ★ backend/alembic/env.py에 import 한 줄 추가
import modules.task.persistence.orm  # noqa: E402, F401
#    ↑ 빼먹으면 Base.metadata가 새 테이블을 모름 → autogenerate 빈 결과

# 3. (필요 시) contract Record + ORM↔Record mapper (boundary)
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
| autogenerate가 빈 revision emit                | 새 entity의 env.py import 누락 (Case B ★)               | env.py에 `import modules.<module>.persistence.orm` 한 줄        |
| `UnicodeDecodeError: 'cp949'`                  | alembic.ini에 한국어 박힘 (Windows configparser locale) | alembic.ini는 ASCII-only 유지                                   |
| autogenerate가 매번 같은 변경 detect           | server default mismatch 등 false positive               | revision 손으로 비우거나 ORM의 `server_default` 명시            |
| `Can't locate revision identified by '<hash>'` | revision file git pull 안 됨 / 누군가 삭제              | `alembic/versions/` sync. **revision file 임의로 지우지 말 것** |
| 동시에 두 PR에서 schema 변경 → branch          | base head 동시 점유                                     | `alembic merge <h1> <h2> -m "merge"` 또는 한 PR rebase          |

## 정책

- Revision file은 git tracked — 모든 머신이 동일 history. 임의 삭제 X
- `alembic_version` 테이블 직접 손대지 말 것 (Alembic이 관리)
- `backend/horibot.db`는 git tracked (현재 컨벤션) — 다른 PC가 같은 캘 데이터로 테스트. schema가 바뀌면 옛 DB도 자동 차분 upgrade
- Squash 안 함 — revision history는 그대로 쌓음
