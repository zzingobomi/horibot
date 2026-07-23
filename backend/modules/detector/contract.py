"""Detector domain — public contract surface.

`Detect Object` = Day-1 primitive (모든 매니퓰레이션 stack 공통, 하드웨어 무관 의미).
구현체(Grounding DINO / YOLO / FoundationPose)는 adapter 뒤 — DSL·Runtime 은
"Detect Object" 만 안다 (backend.md §17.1).

prompt → base frame 3D 후보 **Top-K** (§17.5 ①). 후보별 기하 속성(base_z=position[2],
size_m) 을 제공하고, prior 적용/최종 선택은 소비자(task `SelectTarget(candidates,
prompt, priors)`, §17.5) 책임 — detector 는 "예상 범위" 를 모른다 (계층 분리). multi-view
3D 합의는 후속 (후보 누적 구조만 먼저, §17.5 ③).
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel

from framework.contract.model import DraftModel
from framework.contract.service import declare_service_timeouts


class Detection(BaseModel):
    """검출 후보 — base frame 3D 위치 + 기하 속성 (object-centric, 2026-07-14 재설계).

    position: 물체 **윗면 중심** base frame (m) — 물체 자기 점군(mask→depth→base)의
      윗면 band centroid. base_z: 물체 **아랫면**의 base-z (자기 점군 z 하위
      percentile — 옛 "주변 책상 ring floor 추정" 폐기: 책상 없어도(공중/손) 성립,
      추측이 아니라 관측. grasping.md §1). height = 윗면 − 아랫면.
      ⚠ 단일 뷰는 옆면 depth 가 없어 height 구조적 과소 — 실 height 판정은 멀티뷰
      융합(FUSE_ORIENTED) 결과에서만 의미. score: 신뢰도 0..1.
    bbox_2d: 검출 시점 color 이미지의 픽셀 bbox (x1,y1,x2,y2) — frontend 카메라
      오버레이용 (DetectionsUpdate.image_width/height 기준 좌표).
    """

    prompt: str
    position: tuple[float, float, float]
    score: float
    base_z: float
    height: float
    bbox_2d: tuple[float, float, float, float] | None = None


class DetectionsUpdate(BaseModel):
    """DETECT 1회의 결과 스냅샷 — frontend 카메라 오버레이 (v1 DETECTOR_STATE 계승).

    v1 은 5fps 연속 검출 loop 였으나 v2 detector 는 on-demand — **DETECT 호출
    시마다** publish (task 의 search 자세에서 팔이 멈춰 검출하는 순간 = 오버레이가
    유효한 순간, GPU 추가 비용 0). 팔이 움직이면 stale — frontend 가 timestamp 로
    fade. found=False 여도 빈 candidates 로 publish (오버레이 clear).
    """

    robot_id: str
    seq: int
    timestamp_unix: float
    prompt: str
    image_width: int
    image_height: int
    candidates: list[Detection] = []


class Detector:
    class Service(StrEnum):
        # robot-agnostic (host 당 1, backend.md §2.7) — robot_id 는 req field.
        # 무거운 모델(GDINO)은 1회 로드, 매 요청이 robot_id 로 그 로봇의 camera/캘/TCP 조회.
        DETECT = "srv/detector/detect"  # prompt + robot_id → base 3D 후보 Top-K
        # [DRAFT] OBB(grasp yaw + footprint) 탐색용 — SAM mask → base 점군 → minAreaRect.
        # /dev 로 shape(deg/rad, footprint 정확도) 굳으면 Detection 에 필드 승격 후 DETECT
        # 로 흡수 (DraftModel → StrictModel). §17.5 회전 파지.
        DETECT_ORIENTED = "srv/detector/detect_oriented"
        # [DRAFT] 멀티뷰 관측 융합 — 여러 뷰의 OrientedDetection(points 포함)을
        # 같은 물체끼리 군집 → 점군 합쳐 기하 재계산 (실 height 는 여기서만 —
        # 단일 뷰는 옆면 depth 부재로 과소). 순수 계산 (camera/모델 무관) — 뷰
        # 이동/수집 흐름은 소비자(task) 소유, 기하 산출은 detector 소유.
        FUSE_ORIENTED = "srv/detector/fuse_oriented"
        # [DRAFT] mono 평면 검출 — depth 없는 카메라(omx 웹캠) 전용. mask 픽셀을
        # undistort 후 카메라 ray ∩ (base z=plane_z) 로 역투영해 base XY OBB 를
        # 얻는다 (omx handover — 펜류. projection.plane_points_from_pixels).
        DETECT_PLANAR = "srv/detector/detect_planar"

    class Stream(StrEnum):
        # robot-scoped 키 — payload robot_id 로 framework 라우팅 (host-level 발행,
        # scan BUILD_PROGRESS 동형).
        DETECTIONS = "stream/detector/{robot_id}/detections"
        # [DRAFT] DETECT_ORIENTED 오버레이 — bbox + obb_2d + mask_contour. shape 굳으면
        # DETECTIONS 로 흡수 (OrientedDetectionsUpdate payload = DraftModel).
        DETECTIONS_ORIENTED = "stream/detector/{robot_id}/detections_oriented"


class DetectRequest(BaseModel):
    """멀티 프롬프트 일반형 (2026-07-19) — 한 정지 관측(같은 frame)에서 N 클래스
    동시 검출. prompts 우선, 단일 호출부는 prompt(하위호환) — 서비스가 [prompt]
    로 정규화, 둘 다 비면 found=False. 응답은 flat list 그대로 — 후보마다 자기
    prompt 가 찍혀 온다 (귀속). 추론 전략(prompt 별 단독 N회 vs GDINO 합동
    1-forward)은 detector 내부 설정(deployment detector_joint_inference) — wire
    는 "무엇을 찾을지"만 안다."""

    robot_id: str  # 어느 로봇의 camera/캘/base frame 으로 검출할지 (host당 1 dispatch)
    prompts: list[str] | None = None
    prompt: str | None = None  # 하위호환 (단일) — prompts 없을 때만 사용
    # **프롬프트별** 상위 후보 수 (전체 아님) — 한 prompt 가 상위를 독식해도
    # 다른 prompt 후보가 잘리지 않는다 (§17.5 Top-K). 소비자(task)가 조절.
    top_k: int = 5


class DetectResponse(BaseModel):
    """Top-K 후보 (score desc). found = 후보 ≥1. 최종 선택은 소비자(task SelectTarget)."""

    found: bool
    candidates: list[Detection] = []
    message: str = ""


class OrientedDetection(DraftModel):
    """[DRAFT] Detection + OBB(grasp yaw / footprint). 아직 shape 미확정 (extra=allow).

    Detection 의 모든 필드 + base frame 회전 파지 정보 (base_z/height 의미는
    Detection docstring — object-centric, 자기 점군). /dev 로 검증할 열린 질문:
      grasp_yaw: base Z 회전 rad([-π/2,π/2)) — deg/rad·부호 규약 실물 확인 대상.
      footprint: (long, short) m — mask 윗면 band vs 전체 점군 어느 쪽이 정확한지 tuning.
    굳으면 Detection 에 필드 승격 + base 를 StrictModel 로 교체 (빠뜨린 필드 fail-fast).
    """

    prompt: str
    position: tuple[float, float, float]
    score: float
    base_z: float
    height: float
    grasp_yaw: float
    footprint: tuple[float, float]
    bbox_2d: tuple[float, float, float, float] | None = None
    # 카메라 패널 오버레이(image-space px). obb_2d = base OBB 코너 4개를 픽셀로 reproject
    # (회전 사각형). mask_contour = SAM mask 윤곽 폴리곤 (실루엣). mask bitmap 은 wire 에
    # 안 실음 — 폴리곤(점 수십 개)만 (backend.md 결정). depth 부족 등으로 없으면 None.
    obb_2d: list[tuple[float, float]] | None = None
    mask_contour: list[tuple[float, float]] | None = None
    # 물체 base 점군 (voxel 다운샘플, m) — 멀티뷰 융합(FUSE_ORIENTED) 소스.
    # **서비스 응답에만** 실림 — DETECTIONS_ORIENTED 오버레이 스트림에선 None
    # (mask bitmap 을 wire 스트림에 안 싣는 결정과 같은 근거: 요청자만 받는다).
    points: list[tuple[float, float, float]] | None = None


class DetectOrientedResponse(DraftModel):
    """[DRAFT] detect_oriented 결과 — DETECT_ORIENTED. shape 굳으면 DetectResponse 로 흡수."""

    found: bool
    candidates: list[OrientedDetection] = []
    message: str = ""


class FuseOrientedRequest(DraftModel):
    """[DRAFT] 멀티뷰 관측 융합 — 같은 prompt 로 여러 뷰에서 모은 후보들.

    입력 후보의 points(base 점군)가 융합 소스 — points 없는 후보는 위치 군집에만
    기여. base frame 이라 뷰 간 정렬이 이미 돼 있음 (쌓기만 하면 됨, §5.2).
    """

    candidates: list[OrientedDetection]
    # 같은 물체 판정 XY 반경 (m) — 관측 위치가 이 안이면 한 물체로 군집.
    cluster_eps_m: float = 0.04


class FuseOrientedResponse(DraftModel):
    """[DRAFT] 군집별 융합 결과 (score desc — score = 군집 내 최대).

    융합 후보의 position/base_z/height/footprint/grasp_yaw 는 합친 점군에서
    재계산 — height 가 비로소 실측 (옆면이 다른 뷰에서 채워짐). 뷰 종속 필드
    (bbox_2d/obb_2d/mask_contour)는 None.
    """

    candidates: list[OrientedDetection] = []
    message: str = ""


class DetectPlanarRequest(DraftModel):
    """[DRAFT] mono 평면 검출 — 물체가 base z=plane_z 평면 **위에 놓여 있다**는
    전제가 계약 (테이블 위 얇은 물체 — 펜). depth 를 전혀 안 읽으므로 rgbd 없는
    robot(omx) 에서 동작한다.

    plane_z 소유 = 호출자(task): 테이블 앵커는 1회 설정/측정 config — omx 는
    depth 가 없어 스스로 못 잰다 (docs/omx_handover_prep.md §4 횡단 전제).
    응답 = DetectOrientedResponse. 후보 의미 (mono 정직 표기):
      position = (OBB 중심 XY, plane_z) — **평면 위 발자국 중심** (윗면 아님).
      base_z = plane_z, height = 0.0 (mono 는 높이를 모른다 — 지름 가정은 소비자).
      footprint/grasp_yaw = 평면 투영 OBB (펜 끝점 = 소비자가 position±long/2·yaw).
    """

    robot_id: str
    plane_z: float  # base frame 평면 z (m) — 테이블 앵커
    prompts: list[str] | None = None
    prompt: str | None = None
    top_k: int = 5


class OrientedDetectionsUpdate(DraftModel):
    """[DRAFT] DETECT_ORIENTED 1회의 오버레이 스냅샷 — 카메라 패널 (DetectionsUpdate 의
    oriented 판). bbox + obb_2d(회전 사각형) + mask_contour(실루엣). on-demand publish."""

    robot_id: str
    seq: int
    timestamp_unix: float
    prompt: str
    image_width: int
    image_height: int
    candidates: list[OrientedDetection] = []


# ─── 서비스 기본 timeout (runtime.call 이 timeout 미지정 시 사용) ───

declare_service_timeouts({
    Detector.Service.DETECT: 30.0,  # GDINO 첫 추론이 느릴 수 있음
    Detector.Service.DETECT_ORIENTED: 30.0,
    Detector.Service.FUSE_ORIENTED: 10.0,  # 순수 계산 (점군 수천 수준)
    Detector.Service.DETECT_PLANAR: 30.0,  # DETECT 와 동일 (모델 추론 지배)
})
