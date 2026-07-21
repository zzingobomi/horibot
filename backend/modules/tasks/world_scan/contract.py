from __future__ import annotations

from enum import StrEnum

from framework.contract.model import StrictModel


class WorldScan:
    """World 스캔 task — 로봇이 스캔 자세를 스스로 돌며 캡처→빌드해 3D World
    배경 메시를 만든다 (2026-07-21 pick 편승에서 분리 — docs/pnp_scenario_rework.md
    §3.1). 자율 이동이라 STOP 안전 의무가 있어 task 로 만든다 (TaskRunner 가
    STOP/pause/진행 제공). 산출물(recon)은 scan 모듈이 소유·영속 — 이 task 는
    "돌며 캡처하고 빌드"라는 오케스트레이션만 한다."""

    class Service(StrEnum):
        RUN = "srv/world_scan/run"
        STOP = "srv/world_scan/stop"
        PAUSE = "srv/world_scan/pause"
        RESUME = "srv/world_scan/resume"
        LIST_ROBOTS = "srv/world_scan/list_robots"

    class Stream(StrEnum):
        STATE = "stream/world_scan/{robot_id}/state"
        TRACE = "stream/world_scan/{robot_id}/trace"


class RunRequest(StrictModel):
    # TSDF voxel (m). None = scan 기본(2mm). 스캔 패널 품질 셀렉터가 실을 값 —
    # recon row 에 저장돼 "이 메시가 왜 이 모양" 분석 데이터가 된다.
    voxel_size: float | None = None


class ListRobotsRequest(StrictModel):
    pass


class ListRobotsResponse(StrictModel):
    """task 참여 robot 명부 — 바인딩 SSOT(모듈 TASK_ROBOTS)를 계약으로 노출.

    frontend 는 이 목록으로 robot-scoped 스트림 키의 `{robot_id}` 를 채운다
    (pick_and_place 와 동형 — task 패널은 robot 을 고르지 않고 task 가 알려준다)."""

    robot_ids: list[str]
