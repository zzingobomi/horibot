from __future__ import annotations

from enum import StrEnum

from framework.contract.model import StrictModel


class PickAndPlace:
    class Service(StrEnum):
        RUN = "srv/pick_and_place/run"
        STOP = "srv/pick_and_place/stop"
        PAUSE = "srv/pick_and_place/pause"
        RESUME = "srv/pick_and_place/resume"
        STEP_ONCE = "srv/pick_and_place/step_once"
        RUN_TO = "srv/pick_and_place/run_to"
        TOGGLE_BREAKPOINT = "srv/pick_and_place/toggle_breakpoint"
        LIST_ROBOTS = "srv/pick_and_place/list_robots"
        # 실행 전 정적 프리뷰 (2026-07-14 확정 — tasks/core/preview.py):
        # 소스만 읽는 구조 인덱싱이라 실행/모킹 0. breakpoint/run_to 대상을
        # 실행 전에 보여준다. req/res 는 core 공용 (PreviewRequest/Response).
        PREVIEW = "srv/pick_and_place/preview"

    class Stream(StrEnum):
        STATE = "stream/pick_and_place/{robot_id}/state"
        TRACE = "stream/pick_and_place/{robot_id}/trace"
        MARKERS = "stream/pick_and_place/{robot_id}/markers"


class RunRequest(StrictModel):
    pick_object: str
    place_object: str = ""
    # World 배경 스캔은 2026-07-21 전용 task(world_scan)로 분리 — pick 편승
    # (build_world/world_voxel_size) 폐기. 근거: world 메시는 pick 내 소비자 0
    # (표시용 workcell 자원이 스윕에 우연히 편승했을 뿐), 편승 capture 가
    # 크리티컬 패스에서 7~9s 낭비, best-effort 침묵 실패라 품질 붕괴에 손쓸
    # 방법이 없었음. 이제 스캔 패널의 "자동 스캔"이 소유 (docs/pnp_scenario_rework.md §3.1).


class ListRobotsRequest(StrictModel):
    pass


class ListRobotsResponse(StrictModel):
    """task 참여 robot 명부 — 바인딩 SSOT(모듈 TASK_ROBOTS)를 계약으로 노출.

    frontend 는 이 목록으로 robot-scoped 스트림 키의 `{robot_id}` 를 채운다
    (task 패널은 robot 을 *고르지* 않고, task 가 *알려주는* 사실을 쓴다)."""

    robot_ids: list[str]




class TaskMarker(StrictModel):
    """계획 산출 마커 — 위치 + (optional) 파지/적치 방향 (2026-07-19).

    approach/jaw_axis = base frame 단위벡터 (그리퍼 진입 방향 / 조 이동 축),
    quaternion = TCP 자세. 셋 다 optional — 방향 없는 마커(단순 지점) 호환.
    프론트 TaskMarkersOverlay 가 화살표(approach) + 양방향 바(jaw_axis)로
    "어느 면을 어느 방향으로 무는지"를 렌더. 값 출처 = servo.GraspFamily
    (파지 자세 정본 — antipodal 은 진단 전용 잔재) / PlaceCandidate(적치)."""

    label: str
    position: tuple[float, float, float]
    approach: tuple[float, float, float] | None = None
    jaw_axis: tuple[float, float, float] | None = None
    quaternion: tuple[float, float, float, float] | None = None


class TaskMarkers(StrictModel):
    robot_id: str
    seq: int
    timestamp_unix: float
    markers: list[TaskMarker] = []
