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
    # search 스윕에 편승해 scan 세션(capture→build)을 돌려 3D World 배경을
    # 갱신할지 — 기본 off ("빨리 픽앤플레이스만" 이 기본, 2026-07-18 UX 결정).
    # best-effort: 월드 갱신 실패는 pick 을 죽이지 않는다 (steps.WorldScan).
    build_world: bool = False
    # 월드 갱신 TSDF voxel (m). None = scan 기본(2mm). UI 는 1/2/4/8mm 4단 —
    # 막연한 low/high 가 아니라 실제 조절값을 노출 (recon row 에 저장돼
    # "이 메시가 왜 이 모양" 분석 데이터가 된다). sdf_trunc 는 scan 모듈이 파생.
    world_voxel_size: float | None = None


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
