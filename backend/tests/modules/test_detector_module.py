"""DetectorModule DETECT 단위테스트 — fake runtime 으로 camera/calibration/motion
canned 응답 주입. 실 모듈/하드웨어/모델 없이 DETECT 배선 + 투영 통합 검증 (회사).

의미 있는 검증: mock 검출 bbox + 합성 depth/intrinsic/hand_eye/TCP → base 좌표가
손계산과 일치 + 캘 없을 때 not-found.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import pytest

from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    HandEyeResultData,
    HandEyeResultRecord,
    IntrinsicResultData,
    IntrinsicResultRecord,
)
from modules.camera.contract import (
    Camera,
    CameraDecodedFrame,
    CameraDepthDecodedFrame,
)
from modules.detector.drivers.mock import MockDetectorBackend
from modules.detector.drivers.protocol import RawDetection
from modules.detector.contract import DetectRequest, OrientedDetection
from modules.detector.module import DetectorModule
from modules.motion.contract import Motion, TcpState

_ROBOT = "so101_6dof_0"


@pytest.fixture(autouse=True)
def _dump_to_tmp(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    """detect_oriented 디버그 덤프를 tmp 로 — 테스트가 실 debug/detect/ 에 합성
    (흑색) 이미지를 남기면 실물 덤프 기반 분석이 오염된다 (2026-07-19 실사고:
    joint score 특성화 1차 런 표본에 테스트 잔재 14장이 섞여 통계가 뒤집힘)."""
    import modules.detector.module as dmod

    monkeypatch.setattr(dmod, "_DETECT_DUMP_ROOT", tmp_path / "detect")
    monkeypatch.setattr(dmod, "_DEBUG_DIR", tmp_path)


class _FakeRuntime:
    """key(str) → canned 응답. await runtime.call 만족 (async). publish 캡처."""

    def __init__(self, responses: dict):
        self._responses = responses
        self.published: list[tuple[str, object]] = []

    def publish(self, wire_key, event):  # noqa: ANN001, D102
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):
        return self._responses[str(key)]


def _bundle() -> CalibrationBundle:
    now = datetime.now(UTC)
    return CalibrationBundle(
        robot_id=_ROBOT,
        intrinsic=IntrinsicResultRecord(
            run_id=1,
            robot_id=_ROBOT,
            created_at=now,
            result_data=IntrinsicResultData(
                camera_matrix=[[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]],
                dist_coeffs=[[0.0, 0.0, 0.0, 0.0, 0.0]],
            ),
        ),
        hand_eye=HandEyeResultRecord(
            run_id=1,
            robot_id=_ROBOT,
            created_at=now,
            result_data=HandEyeResultData(
                R_cam2gripper=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                t_cam2gripper=[[0.0], [0.0], [0.0]],
                method="test",
            ),
        ),
    )


def _frame_runtime(w: int = 640, h: int = 480) -> _FakeRuntime:
    """캘/color/depth/tcp canned 응답 rt. detect / detect_oriented 공유.

    mock 후보 2개: 중앙(320,240) 20% → x∈[256,384],y∈[192,288] depth 300 → z 0.3 →
    base [0,0,0.8]. 우상(448,144) → x∈[384,512],y∈[96,192] depth 500 → z 0.5 →
    base [0.1067,-0.08,1.0] (base_z 더 높음 = 후보 구분 근거).
    """
    color = CameraDecodedFrame(
        robot_id=_ROBOT, seq=0, timestamp_unix=0.0,
        ndarray_bytes=np.zeros((h, w, 3), np.uint8).tobytes(), width=w, height=h,
    )
    depth_arr = np.zeros((h, w), np.uint16)
    depth_arr[192:288, 256:384] = 300
    depth_arr[96:192, 384:512] = 500
    depth = CameraDepthDecodedFrame(
        robot_id=_ROBOT, seq=0, timestamp_unix=0.0,
        depth_bytes=depth_arr.tobytes(), width=w, height=h, depth_scale=0.001,
    )
    tcp = TcpState(
        robot_id=_ROBOT, seq=0, timestamp_unix=0.0,
        position=(0.0, 0.0, 0.5), quaternion=(0.0, 0.0, 0.0, 1.0),
        joint_names=["joint1"], joints=[0.0],
    )
    return _FakeRuntime({
        str(Calibration.Service.SNAPSHOT_BUNDLE): _bundle(),
        str(Camera.Service.DECODED_SNAPSHOT): color,
        str(Camera.Service.DEPTH_DECODED_SNAPSHOT): depth,
        str(Motion.Service.TCP_SNAPSHOT): tcp,
    })


async def test_detect_returns_topk_base_positions():
    w, h = 640, 480
    rt = _frame_runtime(w, h)
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())

    res = await mod.detect(DetectRequest(robot_id=_ROBOT, prompt="cube"))
    assert res.found, res.message
    # Top-K: 후보 2개 + score 내림차순 (§17.5 ①)
    assert len(res.candidates) == 2
    assert res.candidates[0].score >= res.candidates[1].score

    best = res.candidates[0]
    # 중앙 균일 윗면 → 윗면 픽셀 centroid ≈ (cx,cy), Z_cam=0.3; hand_eye=I; base t=[0,0,0.5]
    # → base ≈ [0,0,0.8]. (centroid 는 픽셀 그리드 중심이라 bbox 중심과 0.5px≈0.25mm 차 —
    #  균일 블록에선 무해. position = 물체 점군 윗면 band 실측 중심.)
    assert np.allclose(best.position, [0.0, 0.0, 0.8], atol=1e-3), best.position
    assert best.prompt == "cube"
    assert best.score == 0.95
    # object-centric 불변식: base_z = 자기 점군 바닥, height = top − bottom.
    # (균일 평면 mock 이라 단일 뷰 height ≈ 0 — 과소가 정직, 실측은 융합 후.)
    assert isinstance(best.base_z, float) and isinstance(best.height, float)
    assert best.height == max(0.0, best.position[2] - best.base_z)
    assert best.height < 0.005

    # 2등 후보 = 우상 (score 낮음) — Top-K 누적 확인
    second = res.candidates[1]
    assert second.score == 0.60
    assert abs(second.position[2] - 1.0) < 1e-6, second.position

    # bbox_2d 전파 — frontend 카메라 오버레이 소비 (중앙 후보 = x∈[256,384])
    assert best.bbox_2d is not None
    assert best.bbox_2d[0] == 256.0 and best.bbox_2d[2] == 384.0

    # DETECT 마다 DetectionsUpdate publish (v1 DETECTOR_STATE 계승 — 오버레이 wire)
    dets = [e for k, e in rt.published if k.endswith("/detections")]
    assert len(dets) == 1
    upd = dets[0]
    assert upd.robot_id == _ROBOT  # type: ignore[attr-defined]
    assert upd.image_width == w and upd.image_height == h  # type: ignore[attr-defined]
    assert len(upd.candidates) == 2  # type: ignore[attr-defined]


async def test_detect_oriented_returns_obb_candidates():
    # 같은 파이프라인 + mask(=bbox 채운 사각형)→base 점군→minAreaRect OBB.
    rt = _frame_runtime()
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())
    res = await mod.detect_oriented(DetectRequest(robot_id=_ROBOT, prompt="cube"))
    assert res.found, res.message
    assert len(res.candidates) == 2  # detect 와 동일 Top-K
    c = res.candidates[0]
    # detect 와 base 위치/score 동일 (공통 _detect_candidates)
    assert np.allclose(c.position, [0.0, 0.0, 0.8], atol=1e-3), c.position
    assert c.score == 0.95
    # OBB 필드 — grasp yaw(rad) + footprint(long≥short, m)
    assert isinstance(c.grasp_yaw, float)
    assert len(c.footprint) == 2 and c.footprint[0] >= c.footprint[1]
    # 중앙 bbox 128×96px @ z0.3, fx600 → footprint ≈ (0.064, 0.048)m, 축정렬 yaw≈0
    assert abs(c.footprint[0] - 0.064) < 3e-3, c.footprint
    assert abs(c.footprint[1] - 0.048) < 3e-3, c.footprint
    assert abs(c.grasp_yaw) < math.radians(5), c.grasp_yaw
    # 오버레이: obb_2d = 코너 4개(base OBB → 픽셀 reproject). 축정렬 mask 라 다시
    # bbox 로 돌아와야 (round-trip) — 중앙 bbox x∈[256,384], y∈[192,288].
    assert c.obb_2d is not None and len(c.obb_2d) == 4, c.obb_2d
    xs = [p[0] for p in c.obb_2d]
    ys = [p[1] for p in c.obb_2d]
    assert abs(min(xs) - 256) < 3 and abs(max(xs) - 384) < 3, c.obb_2d
    assert abs(min(ys) - 192) < 3 and abs(max(ys) - 288) < 3, c.obb_2d
    # mask_contour = SAM mask 윤곽 (mock = 채운 bbox → 사각형 윤곽 폴리곤)
    assert c.mask_contour is not None and len(c.mask_contour) >= 4, c.mask_contour

    # 오버레이 스냅샷 publish (DETECTIONS_ORIENTED) — 카메라 패널 소비
    ups = [e for k, e in rt.published if k.endswith("/detections_oriented")]
    assert len(ups) == 1
    assert len(ups[0].candidates) == 2  # type: ignore[attr-defined]
    await mod.stop()  # 백그라운드 덤프 드레인 (pending task 경고 방지)


async def test_detect_multi_prompt_tags_and_publishes_combined():
    """★ 멀티 프롬프트 일반화 (2026-07-19): 한 요청(prompts=[cube,box])이 한
    frame 관측에서 두 클래스 후보를 함께 반환 — 후보마다 자기 prompt 귀속 +
    top_k 는 **프롬프트별** + 오버레이는 **프레임당 1건 합본** (latest-wins
    스트림에서 prompt 별 발행은 마지막 것만 남아 pick 이 place 에 덮이는 실사고
    — 2026-07-19). 뒤집으면: 귀속 소실(전부 한 prompt 로 도장) / 전체 top-k 로
    한 prompt 독식 / 오버레이 분할 발행 회귀."""
    rt = _frame_runtime()
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())

    res = await mod.detect(
        DetectRequest(robot_id=_ROBOT, prompts=["cube", "box"], top_k=5)
    )
    assert res.found
    # mock = prompt 당 2 후보 → 총 4, prompt 별 2개씩 (per-prompt Top-K)
    assert len(res.candidates) == 4
    by_prompt: dict[str, int] = {}
    for c in res.candidates:
        by_prompt[c.prompt] = by_prompt.get(c.prompt, 0) + 1
    assert by_prompt == {"cube": 2, "box": 2}
    # 전역 score desc (응답 정렬 계약)
    scores = [c.score for c in res.candidates]
    assert scores == sorted(scores, reverse=True)

    # 오버레이: 프레임당 1건 — 두 prompt 후보가 한 update 에 (덮임 없음)
    ups = [e for k, e in rt.published if k.endswith("/detections")]
    assert len(ups) == 1
    upd = ups[0]
    assert upd.prompt == "cube, box"  # type: ignore[attr-defined]
    assert len(upd.candidates) == 4  # type: ignore[attr-defined]
    assert {c.prompt for c in upd.candidates} == {"cube", "box"}  # type: ignore[attr-defined]


async def test_detect_oriented_multi_prompt_combined_overlay():
    """detect_oriented 도 동일 일반화 — 귀속 + 프레임당 1건 합본 오버레이.
    (task 스윕 통합의 backend 절반 — 후보 분리는 per-candidate 귀속이 유일한
    근거, 카메라 패널은 이 합본에서 prompt 별 best 라벨을 그린다.)"""
    rt = _frame_runtime()
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())
    res = await mod.detect_oriented(
        DetectRequest(robot_id=_ROBOT, prompts=["cube", "box"])
    )
    assert res.found and len(res.candidates) == 4
    assert {c.prompt for c in res.candidates} == {"cube", "box"}
    ups = [e for k, e in rt.published if k.endswith("/detections_oriented")]
    assert len(ups) == 1
    assert {c.prompt for c in ups[0].candidates} == {"cube", "box"}  # type: ignore[attr-defined]
    await mod.stop()  # 백그라운드 덤프 드레인 (pending task 경고 방지)


async def test_detect_prompts_normalization():
    """prompts 정규화 — 공백/중복 제거, 빈 요청은 found=False (prompt 하위호환
    경로는 기존 테스트가 잠금)."""
    rt = _FakeRuntime({})
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())
    res = await mod.detect(DetectRequest(robot_id=_ROBOT, prompts=["  ", ""]))
    assert not res.found
    res2 = await mod.detect(DetectRequest(robot_id=_ROBOT))  # 둘 다 없음
    assert not res2.found


async def test_detect_without_calibration_not_found():
    rt = _FakeRuntime(
        {str(Calibration.Service.SNAPSHOT_BUNDLE): CalibrationBundle(robot_id=_ROBOT)}
    )
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())
    res = await mod.detect(DetectRequest(robot_id=_ROBOT, prompt="cube"))
    assert not res.found
    assert "캘" in res.message


async def test_detect_empty_prompt_not_found():
    rt = _FakeRuntime({})
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())
    res = await mod.detect(DetectRequest(robot_id=_ROBOT, prompt="  "))
    assert not res.found


async def test_start_triggers_backend_preload():
    """start() 가 백그라운드로 backend.preload() 를 호출 — 첫 detect 지연 제거 배선.
    (start 에서 preload 를 빼면 이 assert 가 깨짐 = 의미 있는 회귀.)"""

    class _RecordingBackend:
        def __init__(self) -> None:
            self.preloaded = False

        def detect(
            self, image_bgr: np.ndarray, prompts: Sequence[str], top_k: int
        ) -> list[RawDetection]:
            return []

        def preload(self) -> None:
            self.preloaded = True

    backend = _RecordingBackend()
    mod = DetectorModule(runtime=_FakeRuntime({}), backend=backend)
    await mod.start()
    assert mod._preload_task is not None
    await mod._preload_task  # 백그라운드 preload 완료 대기
    assert backend.preloaded
    await mod.stop()


# ─── FUSE_ORIENTED (멀티뷰 융합 — 순수 계산) ─────────────────────────


def _obs(
    position: tuple[float, float, float],
    points: list[tuple[float, float, float]],
    score: float = 0.9,
) -> OrientedDetection:
    return OrientedDetection(
        prompt="cube", position=position, score=score,
        base_z=position[2], height=0.0, grasp_yaw=0.0,
        footprint=(0.02, 0.02), points=points,
    )


def _cube_face(
    cx: float, cy: float, top_z: float, height: float, *, top: bool,
    edge: float = 0.02, n: int = 8,
) -> list[tuple[float, float, float]]:
    """윗면(top=True) 또는 +x 옆면 점군 — 한 뷰가 보는 면."""
    xs = np.linspace(-edge / 2, edge / 2, n)
    out = []
    for a in xs:
        for b in xs if top else np.linspace(top_z - height, top_z, n):
            if top:
                out.append((cx + a, cy + b, top_z))
            else:
                out.append((cx + edge / 2, cy + a, float(b)))
    return out


async def test_fuse_oriented_merges_views_and_recovers_height():
    """서로 다른 뷰(윗면만 / 옆면 포함) 관측을 융합해 실 height 복원 + 멀리
    떨어진 다른 물체는 별 군집. 단일 뷰 height≈0 → 융합 후 2.3cm (§5.2 핵심)."""
    from modules.detector.contract import FuseOrientedRequest

    mod = DetectorModule(_FakeRuntime({}), MockDetectorBackend())
    top_view = _obs((0.2, 0.1, -0.022), _cube_face(0.2, 0.1, -0.022, 0.023, top=True))
    side_view = _obs(
        (0.205, 0.102, -0.022), _cube_face(0.2, 0.1, -0.022, 0.023, top=False)
    )
    other = _obs((0.5, -0.3, 0.05), _cube_face(0.5, -0.3, 0.05, 0.04, top=True))

    res = await mod.fuse_oriented(
        FuseOrientedRequest(candidates=[top_view, side_view, other])
    )
    assert len(res.candidates) == 2  # 큐브 군집 + 다른 물체
    cube = min(
        res.candidates, key=lambda c: (c.position[0] - 0.2) ** 2 + (c.position[1] - 0.1) ** 2
    )
    assert abs(cube.height - 0.023) < 4e-3, cube.height  # 옆면이 채워져 실측
    assert abs(cube.base_z - (-0.045)) < 4e-3, cube.base_z
    assert cube.points  # 융합 점군 동봉 (재융합/디버그 가능)


async def test_fuse_oriented_empty_and_pointless():
    from modules.detector.contract import FuseOrientedRequest

    mod = DetectorModule(_FakeRuntime({}), MockDetectorBackend())
    empty = await mod.fuse_oriented(FuseOrientedRequest(candidates=[]))
    assert empty.candidates == [] and empty.message

    # 점군 없는 관측만 — 융합 불가, 빈 결과 + 사유
    no_pts = OrientedDetection(
        prompt="cube", position=(0.2, 0.1, 0.0), score=0.9, base_z=0.0,
        height=0.0, grasp_yaw=0.0, footprint=(0.02, 0.02),
    )
    res = await mod.fuse_oriented(FuseOrientedRequest(candidates=[no_pts]))
    assert res.candidates == [] and "점군" in res.message


async def test_detect_roi_cuts_out_of_cell_candidates():
    """작업 셀 ROI 밖 후보 컷 (§3.3) — mock 후보 best(z=0.8)/second(z≈1.0) 중
    셀 z[0.75,0.85] 이 second 만 잘라 1개 남긴다. 오검출(공유기·로봇몸통) 소멸의
    결정적 재현 — 색/score 재튜닝 없이 기하로 컷. ROI 미설정이면 no-op(기존 테스트)."""
    rt = _frame_runtime()
    mod = DetectorModule(
        runtime=rt,
        backend=MockDetectorBackend(),
        workcell={_ROBOT: (-1.0, 1.0, -1.0, 1.0, 0.75, 0.85)},
    )
    res = await mod.detect(DetectRequest(robot_id=_ROBOT, prompt="cube"))
    assert res.found, res.message
    assert len(res.candidates) == 1  # 셀 밖(z≈1.0) 후보 컷
    assert abs(res.candidates[0].position[2] - 0.8) < 1e-3


async def test_detect_roi_cuts_towering_candidate_by_base_z(monkeypatch):
    """★ 공유기 클래스 회귀 (2026-07-21 23:41 실물): 옆 테이블에서 솟은 물체는
    **꼭대기(position z)만 셀에 걸치고 바닥(base_z)은 셀 밖** — position 단독
    판정이 통과시켜 "white small round cube" 0.59 로 servo 관측을 뺏었다.
    base_z 가 셀 바닥 아래면 컷. mock 은 평면 패치(base_z≈top)라 metrics 를
    변조해 '솟은' 기하를 주입 — 꼭대기는 셀 안 그대로, 바닥만 30cm 아래."""
    from modules.detector import module as det_mod

    orig = det_mod.geometry.object_metrics_from_points

    def towering(pts):
        m = orig(pts)
        if m is None:
            return m
        position, bottom, height = m
        return position, bottom - 0.3, height + 0.3  # 바닥만 셀 밖으로

    monkeypatch.setattr(det_mod.geometry, "object_metrics_from_points", towering)
    mod = DetectorModule(
        runtime=_frame_runtime(),
        backend=MockDetectorBackend(),
        workcell={_ROBOT: (-1.0, 1.0, -1.0, 1.0, 0.75, 0.85)},
    )
    res = await mod.detect(DetectRequest(robot_id=_ROBOT, prompt="cube"))
    # 옛 판정(position 만)이면 z=0.8 후보가 남았다 (위 test 와 동일 셀) —
    # base_z(0.5 < 0.75) 조건이 그 후보를 컷해 0건이어야 한다.
    assert res.candidates == [], [
        (c.position[2], c.base_z) for c in res.candidates
    ]
