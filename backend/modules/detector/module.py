"""DetectorModule — `Detect Object` (Day-1 primitive). PC 배치, **robot-agnostic**.

host 당 1 인스턴스 (backend.md §2.7) — 무거운 모델(GDINO)을 1회 로드하고,
매 DETECT 요청의 `req.robot_id` 로 그 로봇의 camera/캘/TCP 를 조회해 dispatch. __init__ 에
robot_id 없음 (framework 계약: host-level = robot_id 미보유).

flow (DETECT): calibration bundle(intrinsic+hand_eye) + camera color/depth snapshot +
motion TCP → adapter 검출(Top-K bbox+mask) → mask→base 점군 → **물체 자기 점군에서**
위치/base_z/height 산출 (object-centric — 주변 바닥 ring 추정 폐기,
grasping.md §1). 최종 선택은 소비자(task).
FUSE_ORIENTED: 멀티뷰 관측(points)을 군집·융합해 기하 재계산 — 실 height 는 여기서만.

모델은 DetectorBackend adapter 뒤 (§0). 투영 수학은 projection.py (결정적).
다른 모듈 호출은 `await self.runtime.call(...)` 로 통일 (backend.md).
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    SnapshotBundleRequest,
)
from modules.camera.contract import (
    Camera,
    CameraDecodedFrame,
    CameraDepthDecodedFrame,
    DecodedSnapshotRequest,
    DepthDecodedSnapshotRequest,
)
from modules.motion.contract import Motion, TcpSnapshotRequest, TcpState

from . import geometry, projection
from .contract import (
    DetectOrientedResponse,
    DetectRequest,
    DetectResponse,
    Detection,
    DetectionsUpdate,
    Detector,
    FuseOrientedRequest,
    FuseOrientedResponse,
    OrientedDetection,
    OrientedDetectionsUpdate,
)
from .drivers.protocol import Bbox, DetectorBackend

logger = logging.getLogger(__name__)

# detect_oriented 디버그 덤프 — SAM mask 원본 픽셀 알파 오버레이 PNG + 후보 점군
# PLY + 메트릭 txt (draft 단계). 패널 오버레이는 contour 폴리곤 근사라 세그멘테이션·
# 점군·base_z/height 정밀 확인은 이 파일들로. 예전엔 마지막 1장만 overwrite 였으나
# (검색 스윕 여러 뷰/집기·놓기 구분 불가) → **backend 세션마다 폴더**를 새로 파고
# 매 호출을 순번으로 쌓는다 (collision 없음, 흐름 그대로 재생). 2026-07-14.
_DEBUG_DIR = Path("debug")
_DETECT_DUMP_ROOT = _DEBUG_DIR / "detect"


def _slug(text: str, limit: int = 40) -> str:
    """파일명 안전 슬러그 — 영숫자/·-_ 만, 공백은 _ (프롬프트 구분용)."""
    out = "".join(c if c.isalnum() or c in "-_" else "_" for c in text.strip())
    return (out[:limit] or "unnamed").strip("_")


def _write_ply(path: Path, points: list[tuple[float, float, float]]) -> None:
    """ASCII PLY 점군 저장 — MeshLab/CloudCompare 로 base_z/height/모양 육안 확인.

    Open3D 로드 없이(무거움) 직접 기록 — voxel 다운샘플된 점(수백~2048)이라 가볍다.
    """
    lines = [
        "ply",
        "format ascii 1.0",
        f"element vertex {len(points)}",
        "property float x",
        "property float y",
        "property float z",
        "end_header",
    ]
    lines.extend(f"{x:.5f} {y:.5f} {z:.5f}" for x, y, z in points)
    path.write_text("\n".join(lines) + "\n")

# 서비스 응답에 싣는 물체 점군 상한 — voxel 다운샘플(3mm) 후에도 큰 물체(천 등)는
# 수만 점이 될 수 있어 stride 로 추가 축소 (융합 기하에 이 밀도면 충분).
_MAX_WIRE_POINTS = 2048


@dataclass(slots=True)
class _Proj:
    """한 프레임의 투영 파라미터 (모든 후보 공통) — intrinsic + TCP pose + hand_eye.
    detect_oriented 가 base OBB 코너를 픽셀로 reproject (오버레이) 할 때 사용."""

    fx: float
    fy: float
    cx: float
    cy: float
    r_be: np.ndarray
    t_be: np.ndarray
    r_ce: np.ndarray
    t_ce: np.ndarray


@dataclass(slots=True)
class _Cand:
    """후보 1개의 base frame 산출 — detect(AABB) / detect_oriented(OBB) 공통 중간형.

    전 기하가 base_points(mask 픽셀의 물체 base 점군)에서 — position=윗면 band
    centroid, base_z=점군 바닥 percentile, height=top−bottom (object-centric).
    depth 무효로 점군을 못 만든 후보는 애초에 제외 (기하 산출 불가 = 후보 아님).
    mask: SAM 픽셀 mask (윤곽 오버레이 소스, detect 는 무시).
    """

    bbox: Bbox
    score: float
    position: tuple[float, float, float]
    base_z: float
    height: float
    base_points: np.ndarray
    mask: np.ndarray


@dataclass(slots=True)
class _DetectResult:
    """_detect_candidates 출력 — 두 서비스가 각자 응답형으로 포맷. color dims 는 빈
    결과에도 오버레이 clear publish 에 필요. proj = 프레임 확보 시만 (없으면 None)."""

    cands: list[_Cand]
    color_w: int
    color_h: int
    message: str
    proj: _Proj | None = None
    img: np.ndarray | None = None  # color BGR — detect_oriented 디버그 덤프용
    # raw 계측 (디버그 덤프 전용) — 다음 분석이 센서 depth 편향 vs 캘/프레임 오차를
    # 분리할 수 있게 원본 depth + 스케일까지 남긴다 (2026-07-15, docs/
    # grasping.md §3 계측 결함 대응). PLY 는 다운샘플·변환 후라
    # 원본 재분석 불가 — raw depth + mask + intrinsic + pose 조합이 있어야 재투영/
    # bias 측정이 된다.
    depth: np.ndarray | None = None  # aligned depth (uint16, color 와 동일 H×W)
    depth_scale: float = 0.001  # depth LSB → m


@publishes(
    (Detector.Stream.DETECTIONS, DetectionsUpdate),
    (Detector.Stream.DETECTIONS_ORIENTED, OrientedDetectionsUpdate),
)
class DetectorModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        backend: DetectorBackend,
    ) -> None:
        self.runtime = runtime
        self._backend = backend
        self._preload_task: asyncio.Task[None] | None = None
        self._detections_seq = 0
        # 디버그 덤프 — backend 세션마다 폴더 1개, 매 호출(detect/fuse)을 순번으로 쌓음.
        self._dump_dir = _DETECT_DUMP_ROOT / time.strftime("%Y%m%d_%H%M%S")
        self._dump_seq = 0

    async def start(self) -> None:
        logger.info("DetectorModule start (host-level)")
        # 백그라운드 preload — boot 를 막지 않는다 (모델 다운로드/로드 수십 초~수 분).
        # blocking 로드 → to_thread. 실패해도 첫 detect 가 lazy 재시도.
        self._preload_task = asyncio.create_task(self._preload())

    async def _preload(self) -> None:
        try:
            await asyncio.to_thread(self._backend.preload)
        except Exception:
            logger.exception("detector backend preload 실패 — 첫 detect 시 재시도")

    async def stop(self) -> None:
        logger.info("DetectorModule stop (host-level)")
        if self._preload_task is not None:
            self._preload_task.cancel()
            self._preload_task = None

    async def _detect_candidates(
        self, robot_id: str, prompt: str, top_k: int
    ) -> _DetectResult:
        """공통 파이프라인 — 캘/frame/backend 검출 → 후보별 base 투영. SSOT.

        detect(AABB) / detect_oriented(OBB) 가 공유. 실패는 빈 cands + 사용자 message
        (found=False 로 매핑). blocking 추론(GPU)만 to_thread, 나머지 순수 계산.
        """
        # 1. intrinsic + hand_eye (같은 캘 출처 → 일관). 없으면 검출 불가.
        # calibration 도 robot-agnostic — 대상 robot 은 req 필드.
        bundle = await self.runtime.call(
            Calibration.Service.SNAPSHOT_BUNDLE,
            SnapshotBundleRequest(robot_id=robot_id),
            CalibrationBundle,
        )
        if bundle.intrinsic is None or bundle.hand_eye is None:
            return _DetectResult([], 0, 0, "intrinsic/hand_eye 캘 없음 — 캘 먼저")

        # 2. color snapshot → 모델 입력
        color = await self.runtime.call(
            Camera.Service.DECODED_SNAPSHOT,
            DecodedSnapshotRequest(),
            CameraDecodedFrame,
            robot_id=robot_id,
        )
        img = np.frombuffer(color.ndarray_bytes, dtype=np.uint8).reshape(
            color.height, color.width, 3
        )

        # 3. depth snapshot → Z (D405 aligned depth)
        depth_f = await self.runtime.call(
            Camera.Service.DEPTH_DECODED_SNAPSHOT,
            DepthDecodedSnapshotRequest(),
            CameraDepthDecodedFrame,
            robot_id=robot_id,
        )
        depth = np.frombuffer(depth_f.depth_bytes, dtype=np.uint16).reshape(
            depth_f.height, depth_f.width
        )

        # 4. adapter 검출 (Top-K RawDetection: bbox+mask+score). blocking(GPU) → to_thread.
        raws = await asyncio.to_thread(self._backend.detect, img, prompt, top_k)
        if not raws:
            return _DetectResult([], color.width, color.height, f"'{prompt}' 감지 실패")

        # 5. 후보 공통 자원 1회 — TCP pose (ee → base) + intrinsic + hand_eye (cam → ee).
        tcp = await self.runtime.call(
            Motion.Service.TCP_SNAPSHOT,
            TcpSnapshotRequest(),
            TcpState,
            robot_id=robot_id,
        )
        r_be = Rotation.from_quat(tcp.quaternion).as_matrix()
        t_be = np.array(tcp.position, dtype=float)
        cm = bundle.intrinsic.result_data.camera_matrix
        fx, fy, cx, cy = cm[0][0], cm[1][1], cm[0][2], cm[1][2]
        r_ce = np.array(bundle.hand_eye.result_data.R_cam2gripper, dtype=float)
        t_ce = np.array(
            bundle.hand_eye.result_data.t_cam2gripper, dtype=float
        ).reshape(3)

        # 6. 후보별 mask→base 점군 → object-centric 기하 (위치/base_z/height 전부
        #    물체 자기 점군에서 — 주변 바닥 ring 추정 폐기). depth 무효로 점군을
        #    못 만든 후보는 기하 산출 불가 → 누락.
        cands: list[_Cand] = []
        for raw in raws:
            base_pts = projection.base_points_from_mask(
                depth, raw.mask, depth_f.depth_scale, fx, fy, cx, cy,
                r_be, t_be, r_ce, t_ce,
            )
            if base_pts is None:
                continue
            metrics = geometry.object_metrics_from_points(base_pts)
            if metrics is None:
                continue
            position, bottom_z, height = metrics
            cands.append(
                _Cand(
                    bbox=raw.bbox,
                    score=float(raw.score),
                    position=position,
                    base_z=bottom_z,
                    height=height,
                    base_points=base_pts,
                    mask=raw.mask,
                )
            )
        msg = "" if cands else "후보 mask depth 전부 무효"
        proj = _Proj(fx, fy, cx, cy, r_be, t_be, r_ce, t_ce)
        return _DetectResult(
            cands, color.width, color.height, msg, proj=proj, img=img,
            depth=depth, depth_scale=depth_f.depth_scale,
        )

    @service(Detector.Service.DETECT)
    async def detect(self, req: DetectRequest) -> DetectResponse:
        prompt = req.prompt.strip()
        if not prompt:
            return DetectResponse(found=False, message="prompt 필요")
        result = await self._detect_candidates(req.robot_id, prompt, req.top_k)
        detections = [
            Detection(
                prompt=prompt,
                position=c.position,
                score=c.score,
                base_z=c.base_z,
                height=c.height,
                bbox_2d=c.bbox,
            )
            for c in result.cands
        ]
        # frontend 카메라 오버레이 — frame 확보 시 결과 스냅샷 publish (빈 결과 포함).
        if result.color_w > 0:
            self._publish_detections(
                req.robot_id, prompt, result.color_w, result.color_h, detections
            )
        if not detections:
            return DetectResponse(found=False, message=result.message)
        # backend 가 score desc 로 줌 → candidates 도 desc 유지 (task SelectTarget 소비).
        return DetectResponse(found=True, candidates=detections)

    @service(Detector.Service.DETECT_ORIENTED)
    async def detect_oriented(self, req: DetectRequest) -> DetectOrientedResponse:
        """[DRAFT] DETECT + mask→base 점군→minAreaRect OBB (grasp yaw + footprint).

        오버레이용 image-space 도 산출: obb_2d = base OBB 코너를 픽셀로 reproject(회전
        사각형), mask_contour = SAM mask 윤곽(실루엣). DETECTIONS_ORIENTED 스트림 publish.
        shape 굳으면 Detection 승격 + DETECT 흡수. depth 점군 부족으로 OBB 못 만든 후보는
        누락 (draft — 실물에서 임계 tuning).
        """
        prompt = req.prompt.strip()
        if not prompt:
            return DetectOrientedResponse(found=False, message="prompt 필요")
        result = await self._detect_candidates(req.robot_id, prompt, req.top_k)
        oriented: list[OrientedDetection] = []
        debug_rows: list[tuple[np.ndarray, list[tuple[float, float]] | None]] = []
        for c in result.cands:
            # 윗면 band 만 — mask 전체는 옆면/배경 bleed 로 footprint 부풀림 + yaw 비틂
            # (geometry.top_face_points 주석, 2026-07-09 실물 확인).
            obb = geometry.obb_from_base_points(
                geometry.top_face_points(c.base_points)
            )
            if obb is None:
                continue
            obb_2d: list[tuple[float, float]] | None = None
            if result.proj is not None:
                p = result.proj
                # base OBB 코너를 물체 윗면 z 평면에 놓고 픽셀로 reproject (오버레이).
                corners = geometry.obb_corners(obb, z=c.position[2])
                px = projection.project_base_to_pixel(
                    corners, p.fx, p.fy, p.cx, p.cy, p.r_be, p.t_be, p.r_ce, p.t_ce
                )
                obb_2d = [(float(u), float(v)) for u, v in px]
            contour = geometry.mask_contour(c.mask)
            mask_contour = (
                [(float(x), float(y)) for x, y in contour]
                if contour is not None
                else None
            )
            debug_rows.append((c.mask, obb_2d))
            # 물체 점군 (voxel 다운샘플 + 상한) — 멀티뷰 융합 소스. 응답에만 싣고
            # 오버레이 스트림에선 뺀다 (_publish_oriented 가 strip).
            ds = geometry.voxel_downsample(c.base_points)
            if len(ds) > _MAX_WIRE_POINTS:
                ds = ds[:: len(ds) // _MAX_WIRE_POINTS + 1]
            oriented.append(
                OrientedDetection(
                    prompt=prompt,
                    position=c.position,
                    score=c.score,
                    base_z=c.base_z,
                    height=c.height,
                    grasp_yaw=obb.yaw_rad,
                    footprint=obb.footprint,
                    bbox_2d=c.bbox,
                    obb_2d=obb_2d,
                    mask_contour=mask_contour,
                    points=[(float(p[0]), float(p[1]), float(p[2])) for p in ds],
                )
            )
        # 오버레이 스냅샷 publish — frame 확보 시 (빈 결과 포함, clear).
        if result.color_w > 0:
            self._publish_oriented(
                req.robot_id, prompt, result.color_w, result.color_h, oriented
            )
        # 디버그 덤프 — 오버레이 PNG + raw(color/depth/mask) + intrinsic/hand_eye/
        #   TCP pose JSON (실패해도 서비스는 무사).
        if result.img is not None and debug_rows:
            try:
                self._dump_debug_image(
                    result.img, oriented, debug_rows,
                    proj=result.proj, depth=result.depth,
                    depth_scale=result.depth_scale,
                )
            except Exception:
                logger.exception("detect_oriented 디버그 덤프 실패 (서비스 영향 없음)")
        if not oriented:
            return DetectOrientedResponse(
                found=False, message=result.message or "OBB 산출 실패"
            )
        return DetectOrientedResponse(found=True, candidates=oriented)

    @service(Detector.Service.FUSE_ORIENTED)
    async def fuse_oriented(self, req: FuseOrientedRequest) -> FuseOrientedResponse:
        """[DRAFT] 멀티뷰 관측 융합 — 순수 계산 (camera/모델/robot 무관).

        위치 XY 군집 → **뷰 간 정합(중심차 평행이동)** → 병합 → object-centric
        기하 재계산. 단일 뷰에서 안 보이던 옆면이 다른 뷰 점군으로 채워져
        height 가 실측이 된다.

        naive vstack 폐기 (2026-07-14 실물 사고): base frame 이라도 뷰 간 검출
        위치가 **계통적으로 1.5~3.3cm** 어긋난다 (스윕 자세별 재현 — STS3215
        백래시 ±0.87°/sag 가 손목 구성마다 다르게 먹는 FK 오차, 캘 σ 7.5mm 밖).
        그냥 겹치면 25mm 큐브가 50×64mm 얼룩이 되고, 얼룩에서 뽑은 가짜
        antipodal 쌍(w=31mm)이 허공을 물었다. 정합 방식·ICP 기각 근거 =
        geometry.align_and_merge_views docstring.
        """
        if not req.candidates:
            return FuseOrientedResponse(candidates=[], message="입력 후보 없음")
        clusters = geometry.cluster_indices_by_xy(
            [c.position for c in req.candidates], req.cluster_eps_m
        )
        fused: list[OrientedDetection] = []
        skipped = 0
        for idx in clusters:
            members = [req.candidates[i] for i in idx]
            with_pts = [m for m in members if m.points]
            if not with_pts:
                skipped += 1  # 점군 없는 관측만으로는 기하 재계산 불가
                continue
            # 정합 병합 — 뷰별 중심차 평행이동 (근거는 geometry 함수 docstring)
            pts = geometry.align_and_merge_views(
                [np.asarray(m.points, dtype=float) for m in with_pts],
                [m.position for m in with_pts],
            )
            metrics = geometry.object_metrics_from_points(pts)
            obb = geometry.obb_from_base_points(geometry.top_face_points(pts))
            if metrics is None or obb is None:
                skipped += 1
                continue
            position, bottom_z, height = metrics
            ds = geometry.voxel_downsample(pts)
            if len(ds) > _MAX_WIRE_POINTS:
                ds = ds[:: len(ds) // _MAX_WIRE_POINTS + 1]
            fused.append(
                OrientedDetection(
                    prompt=members[0].prompt,
                    position=position,
                    score=max(m.score for m in members),
                    base_z=bottom_z,
                    height=height,
                    grasp_yaw=obb.yaw_rad,
                    footprint=obb.footprint,
                    points=[(float(p[0]), float(p[1]), float(p[2])) for p in ds],
                )
            )
        fused.sort(key=lambda c: c.score, reverse=True)
        # 디버그: 융합 점군 덤프 — 멀티뷰가 실제로 옆면을 채웠는지 육안 확인용
        # (단일 뷰 PLY 와 나란히 놓고 비교). 실패해도 서비스는 무사.
        if fused:
            try:
                prefix = self._next_dump_prefix("fuse", fused[0].prompt)
                for i, c in enumerate(fused):
                    if c.points:
                        _write_ply(Path(f"{prefix}_c{i}.ply"), c.points)
                logger.info(
                    "fuse_oriented 디버그 덤프: %s (군집 %d)", prefix, len(fused)
                )
            except Exception:
                logger.exception("fuse_oriented 디버그 덤프 실패 (서비스 영향 없음)")
        msg = f"점군 없는 군집 {skipped}개 제외" if skipped else ""
        return FuseOrientedResponse(candidates=fused, message=msg)

    def _next_dump_prefix(self, kind: str, prompt: str) -> Path:
        """세션 폴더 안 다음 순번 파일 prefix (예: .../0007_det_blue_box). mkdir 포함."""
        self._dump_dir.mkdir(parents=True, exist_ok=True)
        self._dump_seq += 1
        return self._dump_dir / f"{self._dump_seq:04d}_{kind}_{_slug(prompt)}"

    def _dump_debug_image(
        self,
        img_bgr: np.ndarray,
        oriented: list[OrientedDetection],
        rows: list[tuple[np.ndarray, list[tuple[float, float]] | None]],
        *,
        proj: _Proj | None = None,
        depth: np.ndarray | None = None,
        depth_scale: float = 0.001,
    ) -> None:
        """검출 1회 덤프 — mask 알파 오버레이+bbox+OBB+yaw PNG, 후보별 점군 PLY.
        세션 폴더에 순번으로 쌓고(검색 스윕 여러 뷰/집기·놓기 다 보존),
        빠른 확인용 detect_oriented_last.png 도 함께 overwrite.

        패널 오버레이는 contour 폴리곤 근사 — 세그멘테이션이 실제로 어떻게 됐는지
        픽셀 단위 확인은 PNG, base_z/height/모양은 PLY. 후보 수치 메트릭은 **로그**
        (중앙 파일로 수렴)와 `.json` 구조화 사본에 남고, 옛 `.txt` 세 번째 사본은
        폐기했다 (docs/logging.md §3-B).

        **raw 계측(2026-07-15)**: PLY 는 base 변환·다운샘플 후라 센서 원본을 못 되짚는다.
        depth 편향 vs 캘/프레임 오차를 나중에 분리·정량하려면 원본이 있어야 해서
        같이 남긴다: `_color.png`(clean BGR), `_depth.png`(16-bit aligned depth),
        `_mask_c{i}.png`(후보 SAM mask), `.json`(intrinsic+depth_scale+hand_eye
        cam→ee + TCP ee→base + 후보 기하). 재투영·raw depth 측정의 SSOT
        (grasping.md §3). proj/depth 없으면 raw 는 건너뜀.
        """
        canvas = img_bgr.copy()
        cand_summ: list[str] = []
        for i, (det, (mask, obb_2d)) in enumerate(zip(oriented, rows, strict=True)):
            tint = canvas.copy()
            tint[mask] = (239, 70, 217)  # fuchsia (BGR) — 패널 contour 색과 통일
            canvas = cv2.addWeighted(tint, 0.35, canvas, 0.65, 0)
            if det.bbox_2d is not None:
                x1, y1, x2, y2 = (int(round(v)) for v in det.bbox_2d)
                cv2.rectangle(canvas, (x1, y1), (x2, y2), (153, 211, 52), 2)
            if obb_2d is not None:
                pts = np.array(obb_2d, dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(canvas, [pts], True, (11, 158, 245), 3)
                cv2.putText(
                    canvas,
                    f"c{i} yaw {np.degrees(det.grasp_yaw):.0f}deg "
                    f"{det.footprint[0] * 1000:.0f}x{det.footprint[1] * 1000:.0f}mm",
                    (int(obb_2d[0][0]), max(int(obb_2d[0][1]) - 8, 16)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (11, 158, 245),
                    2,
                )
            cand_summ.append(
                f"c{i}: score={det.score:.2f} base_z={det.base_z:.3f}m "
                f"height={det.height * 100:.1f}cm "
                f"pos=({det.position[0]:.3f},{det.position[1]:.3f},"
                f"{det.position[2]:.3f}) footprint="
                f"{det.footprint[0] * 1000:.0f}x{det.footprint[1] * 1000:.0f}mm "
                f"points={len(det.points or [])}"
            )
        prefix = self._next_dump_prefix("det", oriented[0].prompt if oriented else "")
        cv2.imwrite(f"{prefix}.png", canvas)
        # 후보 메트릭 = 로그 서사(중앙 파일 수렴, host/시각 자동 각인) + .json 구조화
        # 사본. 옛 .txt 세 번째 사본은 폐기 (docs/logging.md §3-B).
        logger.info(
            "detect_oriented 후보 %d개 (prompt=%r): %s",
            len(oriented),
            oriented[0].prompt if oriented else "",
            " | ".join(cand_summ) if cand_summ else "없음",
        )
        for i, det in enumerate(oriented):  # 후보별 점군 (base frame, m)
            if det.points:
                _write_ply(Path(f"{prefix}_c{i}.ply"), det.points)
        # raw 계측 — clean color + 16-bit depth + 후보 mask + intrinsic/pose JSON.
        # (센서 depth 편향 vs 캘/프레임 오차 분리용 — docstring/§6.)
        self._dump_debug_raw(prefix, img_bgr, oriented, rows, proj, depth, depth_scale)
        cv2.imwrite(str(_DEBUG_DIR / "detect_oriented_last.png"), canvas)
        logger.info("detect_oriented 디버그 덤프: %s.png (+PLY/raw)", prefix)

    def _dump_debug_raw(
        self,
        prefix: Path,
        img_bgr: np.ndarray,
        oriented: list[OrientedDetection],
        rows: list[tuple[np.ndarray, list[tuple[float, float]] | None]],
        proj: _Proj | None,
        depth: np.ndarray | None,
        depth_scale: float,
    ) -> None:
        """원본 계측 저장 — 재투영/depth-bias 재분석의 입력 (mask→base 를 손으로
        다시 굴려 센서 depth 를 mm 단위로 측정할 수 있게). proj/depth 없으면 skip."""
        if proj is None or depth is None:
            return
        cv2.imwrite(f"{prefix}_color.png", img_bgr)  # clean BGR (tint 전)
        cv2.imwrite(f"{prefix}_depth.png", depth)  # 16-bit aligned depth (LSB)
        for i, (mask, _obb) in enumerate(rows):  # 후보 SAM mask (0/255)
            cv2.imwrite(f"{prefix}_mask_c{i}.png", mask.astype(np.uint8) * 255)
        meta = {
            "prompt": oriented[0].prompt if oriented else "",
            "timestamp_unix": time.time(),
            "depth_scale": float(depth_scale),
            "color_png": Path(f"{prefix.name}_color.png").name,
            "depth_png": Path(f"{prefix.name}_depth.png").name,
            "depth_dtype": str(depth.dtype),
            "depth_shape": list(depth.shape),
            # 좌표 규약 (projection.py): obj_base = R_be·(R_ce·obj_cam + t_ce) + t_be
            "intrinsics": {
                "fx": float(proj.fx), "fy": float(proj.fy),
                "cx": float(proj.cx), "cy": float(proj.cy),
            },
            "hand_eye_cam2ee": {
                "R": np.asarray(proj.r_ce, dtype=float).tolist(),
                "t": np.asarray(proj.t_ce, dtype=float).reshape(3).tolist(),
            },
            "tcp_ee2base": {
                "R": np.asarray(proj.r_be, dtype=float).tolist(),
                "t": np.asarray(proj.t_be, dtype=float).reshape(3).tolist(),
            },
            "candidates": [
                {
                    "index": i,
                    "mask_png": Path(f"{prefix.name}_mask_c{i}.png").name,
                    "score": float(d.score),
                    "position": [float(v) for v in d.position],
                    "base_z": float(d.base_z),
                    "height": float(d.height),
                    "grasp_yaw": float(d.grasp_yaw),
                    "footprint": [float(v) for v in d.footprint],
                    "bbox_2d": (
                        [float(v) for v in d.bbox_2d]
                        if d.bbox_2d is not None else None
                    ),
                    "points": len(d.points or []),
                }
                for i, d in enumerate(oriented)
            ],
        }
        prefix.with_name(prefix.name + ".json").write_text(
            json.dumps(meta, ensure_ascii=False, indent=2)
        )

    def _publish_detections(
        self,
        robot_id: str,
        prompt: str,
        width: int,
        height: int,
        detections: list[Detection],
    ) -> None:
        self.runtime.publish(
            Detector.Stream.DETECTIONS,
            DetectionsUpdate(
                robot_id=robot_id,
                seq=self._detections_seq,
                timestamp_unix=time.time(),
                prompt=prompt,
                image_width=width,
                image_height=height,
                candidates=detections,
            ),
        )
        self._detections_seq += 1

    def _publish_oriented(
        self,
        robot_id: str,
        prompt: str,
        width: int,
        height: int,
        candidates: list[OrientedDetection],
    ) -> None:
        # points 는 스트림에 안 실음 — 오버레이에 불필요한 무거운 원본 (mask
        # bitmap 을 wire 에 안 싣는 backend.md 결정과 같은 근거). 응답에만.
        stripped = [c.model_copy(update={"points": None}) for c in candidates]
        self.runtime.publish(
            Detector.Stream.DETECTIONS_ORIENTED,
            OrientedDetectionsUpdate(
                robot_id=robot_id,
                seq=self._detections_seq,
                timestamp_unix=time.time(),
                prompt=prompt,
                image_width=width,
                image_height=height,
                candidates=stripped,
            ),
        )
        self._detections_seq += 1
