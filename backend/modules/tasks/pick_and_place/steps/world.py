"""World 편승 스캔 — search 스윕 중 3D 배경(World 레이어) 갱신 (best-effort).

pick 의 일부가 아니다 — 어떤 실패도 pick 을 죽이지 않는다. 빌드는 백그라운드
(2026-07-19 최적화 — 실측 근거는 WorldScan docstring).
"""

from __future__ import annotations

import asyncio
import logging
import time

from modules.scan.contract import (
    BuildRequest,
    BuildResponse,
    NewSessionRequest,
    NewSessionResponse,
    Scan,
)
from modules.scan.contract import CaptureRequest as ScanCaptureRequest
from modules.scan.contract import CaptureResponse as ScanCaptureResponse
from modules.tasks.core.context import TaskContext

logger = logging.getLogger(__name__)


class WorldScan:
    """search 스윕 편승 월드 스캔 (RunRequest.build_world) — scan 모듈 재사용.

    목적 = 3D 뷰 배경(World 레이어) 갱신이지 pick 의 일부가 아니다 → **best-effort**:
    어떤 실패도 pick 을 죽이지 않는다 (경고 로그 + 이후 비활성, 취소(STOP)만
    capture 경로에서 그대로 전파).

    **빌드 = 백그라운드 (2026-07-19 최적화)**: 크리티컬 패스에는 capture(정지
    필요, 실측 1.2~1.6s)만 남기고, 전체 재빌드(스캔 누적에 따라 3.8→8.4s 증가)
    는 asyncio task 로 던져 다음 pose 의 MoveJ/검출과 겹친다 — 실측 59.5s 스윕
    중 빌드 몫 ~28s 가 크리티컬 패스에서 사라진다 (품질 무영향: 같은 스캔,
    같은 빌드 — 마지막 결과가 전 스캔 포함). 규약:
    - in-flight 1개 (busy 면 dirty 만 표시 → 완료 후 1회 재킥 — 중간 상태
      일부는 건너뛰지만 각 빌드가 전체 재빌드라 최종 메시는 항상 완전).
    - detect 스윕 종료 시 finalize() — 마지막 캡처까지 포함한 빌드가 반드시
      돌게 킥 (대기 안 함 — task 는 계획/실행으로 진행, 프론트 World 는
      BUILD_PROGRESS done 으로 늦게라도 갱신).
    - STOP/취소: 스윕 취소는 capture 에서 전파. 이미 떠 있는 빌드 task 는
      모션 무관 순수 계산이라 자연 종료에 맡긴다 (상한 = 잔여 ≤2회).
    """

    def __init__(
        self, ctx: TaskContext, robot_id: str, voxel_size: float | None = None
    ) -> None:
        self._ctx = ctx
        self._robot_id = robot_id
        self._voxel = voxel_size  # None = scan 기본 (RunRequest.world_voxel_size)
        self._session_row_id: int | None = None
        self._dead = False  # 첫 실패 후 비활성 (재시도로 pick 을 더 늦추지 않음)
        self._pose_idx = 0  # 계측 로그용 (world_build pose=N)
        self._build_task: asyncio.Task[None] | None = None
        self._build_dirty = False  # busy 중 새 capture 도착 — 완료 후 재킥

    async def start(self) -> None:
        if self._dead:
            return
        try:
            res = await self._ctx.call(
                Scan.Service.NEW_SESSION,
                NewSessionRequest(
                    robot_id=self._robot_id, label="world (pick_and_place)"
                ),
                NewSessionResponse,
            )
            self._session_row_id = res.session.id
            if self._session_row_id is None:
                self._dead = True
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._dead = True
            logger.warning("world 스캔 세션 시작 실패 (%s) — 월드 갱신 없이 계속", e)

    async def capture(self) -> None:
        """현재(정지) 자세에서 capture + 백그라운드 빌드 킥. 실패 = 경고 + 비활성.

        pose 당 capture ms 를 로그에 남긴다 — "편승이 스윕을 얼마나 늦추나"는
        이제 이 값만이다 (build ms 는 백그라운드 로그 — world_build 라인)."""
        if self._dead or self._session_row_id is None:
            return
        self._pose_idx += 1
        try:
            t_cap = time.perf_counter()
            cap = await self._ctx.call(
                Scan.Service.CAPTURE,
                ScanCaptureRequest(session_row_id=self._session_row_id),
                ScanCaptureResponse,
                timeout=20.0,
            )
            cap_ms = (time.perf_counter() - t_cap) * 1000.0
            if not cap.accepted:
                logger.warning(
                    "world 스캔 capture 거부 (%s) — 이 pose 건너뜀", cap.message
                )
                return
            logger.info(
                "world_capture pose=%d: %.0fms (빌드는 백그라운드)",
                self._pose_idx, cap_ms,
            )
            self._kick_build()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._dead = True
            logger.warning("world 스캔 실패 (%s) — 이후 pose 비활성, pick 계속", e)

    def finalize(self) -> None:
        """스윕 종료 — 마지막 캡처까지 포함한 빌드 보장 킥 (대기 안 함)."""
        self._kick_build()

    def _kick_build(self) -> None:
        if self._dead or self._session_row_id is None:
            return
        if self._build_task is not None and not self._build_task.done():
            self._build_dirty = True  # 완료 후 1회 재킥 (전체 재빌드라 충분)
            return
        self._build_dirty = False
        self._build_task = asyncio.create_task(self._build_bg())

    async def _build_bg(self) -> None:
        """전체 재빌드 1회 (백그라운드) — 실패는 경고 + 비활성 (기존 계약),
        완료 시 dirty 면 재킥 (그 사이 캡처분 포함 최신화)."""
        assert self._session_row_id is not None
        try:
            t_build = time.perf_counter()
            build = await self._ctx.call(
                Scan.Service.BUILD,
                BuildRequest(
                    session_row_id=self._session_row_id, voxel_size=self._voxel
                ),
                BuildResponse,
                timeout=120.0,  # scan 은 timeout 미선언 — DEFAULT 로는 빌드가 잘림
            )
            build_ms = (time.perf_counter() - t_build) * 1000.0
            if not build.accepted:
                logger.warning(
                    "world 빌드 거부 (%s) — 다음 킥에서 재시도", build.message
                )
            else:
                rec = build.reconstruction
                logger.info(
                    "world_build (bg): build %.0fms"
                    " (scans=%d, %d verts, %d tris, voxel=%.1fmm)",
                    build_ms,
                    rec.n_scans if rec else -1,
                    rec.vertex_count if rec else -1,
                    rec.triangle_count if rec else -1,
                    (rec.voxel_size if rec else 0.0) * 1000.0,
                )
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._dead = True
            logger.warning("world 빌드 실패 (%s) — 월드 갱신 비활성, pick 계속", e)
        finally:
            if self._build_dirty and not self._dead:
                self._build_dirty = False
                self._build_task = asyncio.create_task(self._build_bg())
