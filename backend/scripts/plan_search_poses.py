"""책상 커버 최적 search 자세 플래너 (offline, backend 이월 없음 — 2026-07-18 신설).

목적: pick_and_place 의 search 스윕 자세(waypoint 'search' 그룹)를 실측 데이터로
최적화. 사람이 눈대중으로 티칭한 3자세(left/center/right)는 커버·정합 그래프가
빈약하다 (2026-07-18 A/B 실측: recon edge 2개, loop closure fitness 0.08 탈락).

방법 (추측 0 — 전부 실 자산에서):
  1) 기존 3자세를 FK → 카메라 pose → **원거리 관측 대역(elevation·거리) 실측**.
     사용자가 "멀리서 보는 이유는 추후 hand-eye 고려" 라 명시 → 이 대역은
     최적화가 못 건드리는 하드 제약 (가까이 가면 depth 좋아진다고 낮추지 않음).
  2) 최근 reconstruction(.ply)에서 **테이블 평면 RANSAC** → ROI(관측된 책상 범위).
  3) ROI 격자 look-point 마다 (실측 elevation·거리, azimuth=base→point) look-at
     카메라 pose 구성 → target TCP = T_base_cam · inv(hand_eye) → **IK + 바닥 충돌
     검증**. 뒤(벽)는 제외 (base 전방 반평면 밖 look-point 컷).
  4) 각 유효 후보의 **커버 = ROI 셀을 이미지에 투영해 in-FOV + depth 범위**.
     greedy set-cover 로 최소 자세 + 인접 중첩(정합용) 확보, azimuth 순 정렬
     (serpentine — 인접 자세 발자국이 겹쳐 loop closure 생성).
  5) dry-run = 리포트 + top-down PNG (기존 vs 제안 커버). --commit 만 DB 기록:
     **비파괴** — 원본 자세는 '{backup}'(기본 search_manual) 그룹에 먼저 보존한 뒤
     대상 그룹('search' = pick_and_place 가 읽는 이름)에 6자세 기록. A/B 되돌리기
     = --restore. 자동 생성 자세의 첫 실물 방문은 감독 필요.

pick_and_place 는 **이름이 'search' 인 그룹**을 읽는다 (steps._SEARCH_GROUP 하드코딩
— 별도 default 플래그/DB 필드 없음). 그래서 "default = 그룹 이름 search". 6자세를
search 에 넣으면 pnp 가 바로 쓰고, 원본은 search_manual 에 안전 보존된다.

backend(runtime) 떠 있으면 RDB lock 충돌 — 종료 후 실행 (calibrate_offline 동일).

CLI:
  uv run --no-sync python scripts/plan_search_poses.py            # dry-run + PNG
  uv run --no-sync python scripts/plan_search_poses.py --commit   # search←6, 원본→search_manual
  uv run --no-sync python scripts/plan_search_poses.py --restore  # A/B 되돌리기 (원본 복원)
  ... --robot so101_6dof_0 --max-poses 8 --group search --backup-group search_manual
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from apps.config import _ROBOT_DIR  # noqa: E402
from apps.main import load_configs  # noqa: E402
from infra.database.boot import open_database  # noqa: E402
from infra.object_store.filesystem import FilesystemObjectStore  # noqa: E402
from modules.calibration.persistence.repository import (  # noqa: E402
    CalibrationRepository,
)
from modules.motion import units  # noqa: E402
from modules.motion.adapters.pybullet import PybulletKinematics  # noqa: E402
from modules.motion.kinematics import Kinematics  # noqa: E402
from modules.motion.kinematics_builder import build_calibrated_kinematics  # noqa: E402
from modules.motor.contract import MotorKind  # noqa: E402
from modules.scan.persistence.repository import ScanRepository  # noqa: E402
from modules.waypoint.contract import WaypointGroupRecord, WaypointRecord  # noqa: E402
from modules.waypoint.persistence.repository import WaypointRepository  # noqa: E402

logger = logging.getLogger("plan_search")

_SEARCH_GROUP = "search"
_PLANE_DIST = 0.008  # RANSAC 평면 inlier 임계 (8mm)
_ROI_CELL = 0.02  # ROI 커버 격자 (2cm)
_DEPTH_MIN = 0.12  # 커버 판정 depth 하한 (D405 근거리)
_DEPTH_MAX = 0.50  # = build depth_trunc (그 너머는 빌드에서 버려짐)


@dataclass
class CamPose:
    """카메라 optical pose (base frame) + 그걸 만든 관절해."""

    joints: list[float]
    C: np.ndarray  # 3, camera position
    R: np.ndarray  # 3x3, base←cam (열 = cam x/y/z 축의 base 표현)
    name: str = ""
    cov: np.ndarray | None = None  # ROI 커버 마스크 (plan 중 채움)


@dataclass
class Intr:
    w: int
    h: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class Kin:
    kinematics: Kinematics
    arm_specs: list
    joint_offsets: list[float] | None
    t_tcp_cam: np.ndarray  # 4x4 hand_eye (TCP←cam)


def load_kin(robots, cal_repo, robot_id: str) -> Kin:
    r = robots[robot_id]
    arm_specs = [m for m in r.motors if m.kind != MotorKind.GRIPPER]
    urdf = _ROBOT_DIR / r.type / "urdf" / f"{r.type}.urdf"
    bundle = cal_repo.get_active_bundle(robot_id)
    if bundle.hand_eye is None:
        raise SystemExit("hand_eye 캘 없음 — 플래너 불가 (캘 먼저)")
    t_tcp_cam = np.eye(4)
    t_tcp_cam[:3, :3] = np.array(bundle.hand_eye.result_data.R_cam2gripper, float)
    t_tcp_cam[:3, 3] = np.array(
        bundle.hand_eye.result_data.t_cam2gripper, float
    ).reshape(3)
    built = build_calibrated_kinematics(
        urdf, robot_id, arm_specs, bundle, PybulletKinematics
    )
    built.kinematics.initialize()
    logger.info("kin 구성 (캘 적용: %s)", "+".join(built.applied) or "무보정")
    return Kin(built.kinematics, arm_specs, built.joint_offsets, t_tcp_cam)


def cam_pose_from_joints(kin: Kin, joints: list[float], name: str = "") -> CamPose:
    """관절 → 카메라 optical pose (base frame). T_base_cam = T_base_tcp · hand_eye."""
    rot, pos = kin.kinematics.fk_to_matrix(joints)
    t_base_tcp = np.eye(4)
    t_base_tcp[:3, :3] = np.array(rot, float)
    t_base_tcp[:3, 3] = np.array(pos, float)
    t_base_cam = t_base_tcp @ kin.t_tcp_cam
    return CamPose(joints=list(joints), C=t_base_cam[:3, 3], R=t_base_cam[:3, :3], name=name)


def raw_to_arm_rad(kin: Kin, raw: list[int], ids: list[int]) -> list[float]:
    by_id = dict(zip(ids, raw))
    rads = [units.raw_to_rad(by_id[m.id], m) for m in kin.arm_specs]
    if kin.joint_offsets is not None:
        rads = [a + b for a, b in zip(rads, kin.joint_offsets)]
    return rads


def table_plane(store, scan_repo, robot_id: str) -> tuple[float, np.ndarray, Intr]:
    """최근 recon .ply → 테이블 평면 RANSAC (plane_z, normal) + 대표 intrinsic."""
    sessions = scan_repo.list_sessions(robot_id)
    recon_row = None
    scans: list = []
    for s in sorted(sessions, key=lambda x: x.id or 0, reverse=True):
        recs = scan_repo.list_reconstructions(s.id)
        if recs:
            recon_row = max(recs, key=lambda r: r.n_scans)
            scans = scan_repo.list_scans(s.id)
            break
    if recon_row is None or not scans:
        raise SystemExit("reconstruction 없음 — scan 세션 먼저 (플래너는 실 책상 필요)")
    import os
    import tempfile

    fd, p = tempfile.mkstemp(suffix=".ply")
    os.close(fd)
    Path(p).write_bytes(store.get(recon_row.blob_key))
    mesh = o3d.io.read_triangle_mesh(p)
    Path(p).unlink(missing_ok=True)
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    model, inliers = pcd.segment_plane(_PLANE_DIST, ransac_n=3, num_iterations=2000)
    a, b, c, d = model
    n = np.array([a, b, c], float)
    n = n / np.linalg.norm(n)
    if n[2] < 0:
        n, d = -n, -d
    inlier_pts = np.asarray(pcd.points)[inliers]
    plane_z = float(np.median(inlier_pts[:, 2]))
    s0 = scans[0]
    intr = Intr(s0.width, s0.height, s0.fx, s0.fy, s0.cx, s0.cy)
    logger.info(
        "테이블 평면: z=%.3fm normal=(%.2f,%.2f,%.2f) inlier %d/%d, "
        "XY bbox x[%.2f,%.2f] y[%.2f,%.2f]",
        plane_z, *n, len(inliers), len(pcd.points),
        inlier_pts[:, 0].min(), inlier_pts[:, 0].max(),
        inlier_pts[:, 1].min(), inlier_pts[:, 1].max(),
    )
    return plane_z, inlier_pts, intr


def optical_axis(pose: CamPose) -> np.ndarray:
    return pose.R[:, 2]  # cam +Z (optical forward) in base frame


def look_point(pose: CamPose, plane_z: float) -> np.ndarray | None:
    """optical axis 와 테이블 평면(z=plane_z)의 교점."""
    axis = optical_axis(pose)
    if abs(axis[2]) < 1e-6:
        return None
    t = (plane_z - pose.C[2]) / axis[2]
    if t <= 0:
        return None
    return pose.C + t * axis


def characterize(poses: list[CamPose], plane_z: float) -> tuple[float, float, np.ndarray]:
    """기존 자세 → (평균 elevation rad, 평균 거리 m, 전방 단위벡터 XY). 관측 대역 SSOT."""
    elevs, dists, fronts = [], [], []
    for pose in poses:
        lp = look_point(pose, plane_z)
        if lp is None:
            continue
        v = pose.C - lp  # look-point → camera
        dist = float(np.linalg.norm(v))
        elev = math.asin(max(-1.0, min(1.0, v[2] / dist)))  # 수평 위로
        horiz = np.array([lp[0], lp[1]])
        fronts.append(horiz / (np.linalg.norm(horiz) + 1e-9))
        elevs.append(elev)
        dists.append(dist)
        logger.info(
            "  %-14s cam=(%.2f,%.2f,%.2f) look=(%.2f,%.2f) dist=%.3fm elev=%.1f°",
            pose.name, *pose.C, lp[0], lp[1], dist, math.degrees(elev),
        )
    front = np.mean(fronts, axis=0)
    front = front / (np.linalg.norm(front) + 1e-9)
    return float(np.mean(elevs)), float(np.mean(dists)), front


def interp_joints(knots: list[list[float]], j1: float) -> list[float]:
    """티칭 자세들을 J1(방위) 축으로 piecewise-linear 보간 → J1=target 자세.

    티칭 자세는 이미 관측 대역(elev·거리)을 J1 을 따라 그린다 — 그 사이 보간은
    대역 위에 머물고 도달성이 (외삽 아니면) 사실상 보장. 관절 N개 각각 독립 보간."""
    ks = sorted(knots, key=lambda k: k[0])
    xs = [k[0] for k in ks]
    out = [j1]
    for j in range(1, len(ks[0])):
        out.append(float(np.interp(j1, xs, [k[j] for k in ks])))
    return out


def reachable(kin: Kin, joints: list[float]) -> bool:
    """보간 자세 검증 = self/바닥 충돌 없음.

    URDF joint_limits 재검은 **의도적으로 안 한다** — 티칭 자세가 실물에선
    도달하지만 URDF 모델 리밋을 살짝 넘는 자리가 있다 (실측: so101 J4=1.564rad
    이 URDF +1.518 을 2.6° 초과, 세 티칭 자세 공통 — URDF 리밋이 보수적).
    보간은 그 실증된 유효 자세들 **사이**라 물리적으로 유효하다. 기하 충돌만이
    보간이 새로 만들 수 있는 위험이므로 그것만 게이트 (J1 범위는 호출부가
    실증 envelope 로 가둠 = 외삽 없음)."""
    return not kin.kinematics.self_collision(joints) and not kin.kinematics.floor_collision(
        joints, 0.0
    )


def coverage(pose: CamPose, cells: np.ndarray, intr: Intr) -> np.ndarray:
    """ROI 셀(N×3, base) → 이 카메라가 보는 셀 마스크 (in-FOV + depth 범위)."""
    R_cb = pose.R.T  # cam←base
    rel = cells - pose.C
    cam = rel @ R_cb.T  # N×3 in cam optical frame
    z = cam[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = intr.fx * cam[:, 0] / z + intr.cx
        v = intr.fy * cam[:, 1] / z + intr.cy
    return (
        (z > _DEPTH_MIN) & (z < _DEPTH_MAX)
        & (u >= 0) & (u < intr.w) & (v >= 0) & (v < intr.h)
    )


@dataclass
class Plan:
    chosen: list[CamPose] = field(default_factory=list)
    roi_cells: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    covered_existing: np.ndarray = field(default_factory=lambda: np.empty(0, bool))
    covered_new: np.ndarray = field(default_factory=lambda: np.empty(0, bool))


def _overlap(a: np.ndarray, b: np.ndarray) -> float:
    """두 커버 마스크의 겹침 = |A∩B| / min(|A|,|B|) (인접 정합용 중첩률)."""
    inter = int((a & b).sum())
    m = min(int(a.sum()), int(b.sum()))
    return inter / m if m else 0.0


def loop_pairs(poses: list[CamPose]) -> int:
    """예상 loop closure 수 — 비인접 쌍 중 카메라 근접 + 발자국 중첩(build.py 게이트)."""
    n = 0
    for i in range(len(poses)):
        for j in range(i + 2, len(poses)):
            if float(np.linalg.norm(poses[i].C - poses[j].C)) > 0.30:
                continue
            ci, cj = poses[i].cov, poses[j].cov
            if ci is not None and cj is not None and _overlap(ci, cj) > 0.1:
                n += 1
    return n


def plan(
    kin: Kin, existing: list[CamPose], plane_z: float, inliers: np.ndarray,
    intr: Intr, max_poses: int,
) -> Plan:
    elev, dist, front = characterize(existing, plane_z)
    logger.info(
        "관측 대역(제약): elev=%.1f° dist=%.3fm front=(%.2f,%.2f)",
        math.degrees(elev), dist, *front,
    )
    # ROI = inlier 근처 셀 (스캔이 본 책상 = 전방; 벽 뒤는 데이터 없음).
    from scipy.spatial import KDTree

    x0, x1 = inliers[:, 0].min(), inliers[:, 0].max()
    y0, y1 = inliers[:, 1].min(), inliers[:, 1].max()
    gx, gy = np.meshgrid(
        np.arange(x0, x1 + _ROI_CELL, _ROI_CELL),
        np.arange(y0, y1 + _ROI_CELL, _ROI_CELL),
    )
    roi = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, plane_z)], axis=1)
    tree = KDTree(inliers[:, :2])
    roi = roi[tree.query(roi[:, :2], k=1)[0] < _ROI_CELL]
    logger.info("ROI 셀 %d개 (%.0f cm 격자)", len(roi), _ROI_CELL * 100)

    cov_exist = np.zeros(len(roi), bool)
    for pose in existing:
        pose.cov = coverage(pose, roi, intr)
        cov_exist |= pose.cov

    # 티칭 자세를 J1(방위) 노트로 → 조밀 J1 그리드 후보 (보간, 관측 대역 위).
    knots = [p.joints for p in existing]
    j1s = sorted(p.joints[0] for p in existing)
    fine = np.linspace(j1s[0], j1s[-1], 80)
    cand: list[CamPose] = []
    for j1 in fine:
        joints = interp_joints(knots, float(j1))
        if not reachable(kin, joints):
            continue
        pose = cam_pose_from_joints(kin, joints)
        pose.cov = coverage(pose, roi, intr)
        if pose.cov.sum() > 0:
            cand.append(pose)
    logger.info("J1 스윕 후보: %d/%d 도달가능·커버>0", len(cand), len(fine))

    # 선택 = 도달가능 J1 범위 **균등 분할** (max_poses 개).
    #
    # 왜 균등 분할인가 (2026-07-18 실측): 커버는 이미 포화 (D405 0.27m 발자국
    # ~0.5m > 책상 34×74cm → 어느 자세든 테이블 거의 전부를 봄, 중첩 항상 ~100%).
    # 커버 최적화는 무의미 — 메시 품질의 레버는 **pose graph 밀도**다: 자세가
    # 촘촘하면 (a) loop closure 후보 쌍↑ (전역 최적화 구속↑), (b) 인접 정합
    # baseline↓ (A/B 실측 right→center 큰 점프는 fitness 0.25 로 실패, 작은
    # 스텝은 안정 정합). max_poses 로 밀도↔시간 트레이드오프.
    if not cand:
        return Plan([], roi, cov_exist, np.zeros(len(roi), bool))
    cand.sort(key=lambda pp: pp.joints[0])
    j1_lo, j1_hi = cand[0].joints[0], cand[-1].joints[0]
    chosen: list[CamPose] = []
    used: set[int] = set()
    for jt in np.linspace(j1_lo, j1_hi, max(2, max_poses)):
        idx = int(np.argmin([abs(c.joints[0] - jt) for c in cand]))
        if idx not in used:
            used.add(idx)
            chosen.append(cand[idx])

    covered = np.zeros(len(roi), bool)
    covs: list[np.ndarray] = []
    for i, pose in enumerate(chosen):
        pose.name = f"search_auto_{i}"
        assert pose.cov is not None  # cand 구성에서 cov.sum()>0 만 채택
        covered |= pose.cov
        covs.append(pose.cov)
    ovs = [_overlap(covs[i], covs[i + 1]) for i in range(len(covs) - 1)]
    logger.info(
        "선택 %d자세 — 인접 중첩 %s, 예상 loop closure %d쌍",
        len(chosen), ", ".join(f"{o:.0%}" for o in ovs), loop_pairs(chosen),
    )
    for pose in chosen:
        lp = look_point(pose, plane_z)
        logger.info(
            "  %s J1=%.2f cam=(%.2f,%.2f,%.2f) look=(%.2f,%.2f)",
            pose.name, pose.joints[0], *pose.C,
            *(lp[:2] if lp is not None else (0, 0)),
        )
    return Plan(chosen, roi, cov_exist, covered)


def render(plan_: Plan, existing: list[CamPose], plane_z: float, png: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    matplotlib.rcParams["font.family"] = "Malgun Gothic"
    matplotlib.rcParams["axes.unicode_minus"] = False
    roi = plan_.roi_cells
    fig, axes = plt.subplots(1, 2, figsize=(15, 7), dpi=130)

    def draw(ax, mask, poses, title):
        ax.scatter(roi[~mask, 0], roi[~mask, 1], c="#d0d0d0", s=6, marker="s")
        ax.scatter(roi[mask, 0], roi[mask, 1], c="#2a9d8f", s=6, marker="s")
        for pose in poses:
            lp = look_point(pose, plane_z)
            ax.plot([0], [0], "k^", ms=9)
            if lp is not None:
                ax.plot([pose.C[0], lp[0]], [pose.C[1], lp[1]], "-", c="#e76f51", lw=0.8)
                ax.plot(pose.C[0], pose.C[1], "o", c="#e76f51", ms=6)
                ax.annotate(pose.name.replace("search_", ""), (pose.C[0], pose.C[1]),
                            fontsize=6, ha="center")
        pct = 100.0 * mask.sum() / max(1, len(mask))
        ax.set_title(f"{title} — 커버 {pct:.0f}% ({mask.sum()}/{len(mask)} 셀)", fontsize=10)
        ax.set_aspect("equal")
        ax.set_xlabel("base X (m)", fontsize=8)
        ax.set_ylabel("base Y (m)", fontsize=8)

    draw(axes[0], plan_.covered_existing, existing, f"기존 {len(existing)}자세 (티칭)")
    draw(axes[1], plan_.covered_new, plan_.chosen, f"제안 {len(plan_.chosen)}자세 (플래너)")
    fig.suptitle("search 자세 커버 비교 — 책상 ROI top-down (▲=so101 base)")
    fig.tight_layout()
    fig.savefig(png)
    logger.info("PNG: %s", png)


def _ensure_group(waypoint_repo, robot_id: str, name: str) -> int:
    grp = waypoint_repo.get_group_by_name(robot_id, name)
    if grp is None:
        grp = waypoint_repo.insert_group(
            WaypointGroupRecord(robot_id=robot_id, name=name)
        )
    assert grp.id is not None
    return grp.id


def commit(
    waypoint_repo, robot_id: str, chosen: list[CamPose], kin: Kin,
    *, group: str, backup: str,
) -> None:
    """제안 자세를 '{group}' 그룹에 기록 (pick_and_place 가 읽는 그룹 = 이름 SSOT).

    **비파괴**: group 에 기존 멤버가 있으면 먼저 '{backup}' 그룹에 원본을 보존한다
    (waypoint 레코드는 그대로 — 멤버십만 복제). backup 이 이미 차 있으면 원본은
    최초 1회만 보존하고 재백업 skip (덮어쓰기 방지). A/B 되돌리기 = --restore.
    joint_names = arm_specs 순 (waypoint 계약)."""
    names = [m.name for m in kin.arm_specs]
    gid = _ensure_group(waypoint_repo, robot_id, group)
    existing = waypoint_repo.list_group_members(gid)
    if existing:
        bid = _ensure_group(waypoint_repo, robot_id, backup)
        if waypoint_repo.list_group_members(bid):
            logger.info("'%s' 백업 이미 존재 — 원본 보존됨 (재백업 skip)", backup)
        else:
            for wp in existing:
                if wp.id is not None:
                    waypoint_repo.add_member(bid, wp.id)
            logger.info(
                "원본 %d자세 → '%s' 백업 (%s)", len(existing), backup,
                ", ".join(w.name for w in existing),
            )
    for wp in existing:
        if wp.id is not None:
            waypoint_repo.remove_member(gid, wp.id)
    for pose in chosen:
        rec = waypoint_repo.insert_waypoint(
            WaypointRecord(
                robot_id=robot_id, name=pose.name,
                joint_values=[float(x) for x in pose.joints],
                joint_names=names, created_at=datetime.now(UTC),
            )
        )
        assert rec.id is not None
        waypoint_repo.add_member(gid, rec.id)
    logger.info("commit: '%s' 그룹 = %d 자세 (%s)", group, len(chosen),
                ", ".join(p.name for p in chosen))


def restore(waypoint_repo, robot_id: str, *, group: str, backup: str) -> None:
    """'{backup}' 원본 자세를 '{group}' 로 복원 (A/B 되돌리기). group 은 backup
    멤버십으로 교체 (auto 자세 waypoint 레코드는 남되 그룹에서만 빠짐)."""
    bgrp = waypoint_repo.get_group_by_name(robot_id, backup)
    members = (
        waypoint_repo.list_group_members(bgrp.id)
        if bgrp and bgrp.id is not None else []
    )
    if not members:
        raise SystemExit(f"'{backup}' 백업 그룹 없음/빔 — 복원할 원본이 없습니다")
    gid = _ensure_group(waypoint_repo, robot_id, group)
    for wp in waypoint_repo.list_group_members(gid):
        if wp.id is not None:
            waypoint_repo.remove_member(gid, wp.id)
    for wp in members:
        if wp.id is not None:
            waypoint_repo.add_member(gid, wp.id)
    logger.info("restore: '%s' ← '%s' 원본 %d자세 (%s)", group, backup,
                len(members), ", ".join(w.name for w in members))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="so101_6dof_0")
    ap.add_argument("--host", default="pc")
    ap.add_argument(
        "--max-poses", type=int, default=6,
        help="스윕 자세 수 (많을수록 pose graph 촘촘=메시↑, run 시간↑)",
    )
    ap.add_argument("--png", default=str(Path(__file__).parent / "search_plan.png"))
    ap.add_argument("--commit", action="store_true")
    ap.add_argument(
        "--group", default=_SEARCH_GROUP,
        help=f"기록 대상 그룹 (pick_and_place 가 읽는 이름 = '{_SEARCH_GROUP}')",
    )
    ap.add_argument(
        "--backup-group", default="search_manual",
        help="commit 시 원본을 보존할 그룹 (비파괴 — A/B --restore 로 되돌림)",
    )
    ap.add_argument(
        "--restore", action="store_true",
        help="백업 원본을 대상 그룹으로 복원 (A/B 되돌리기 — plan 안 함)",
    )
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    deploy, robots = load_configs(args.host)
    if not deploy.rdb_uri or not deploy.object_uri:
        raise SystemExit(f"host {args.host!r} 에 rdb_uri/object_uri 없음")
    _engine, sf = open_database(deploy.rdb_uri)
    scan_repo = ScanRepository(sf)
    cal_repo = CalibrationRepository(sf)
    wp_repo = WaypointRepository(sf)
    store = FilesystemObjectStore(deploy.object_uri)

    if args.restore:
        restore(wp_repo, args.robot, group=args.group, backup=args.backup_group)
        return

    kin = load_kin(robots, cal_repo, args.robot)
    try:
        plane_z, inliers, intr = table_plane(store, scan_repo, args.robot)
        # 기존 search 그룹 자세 FK
        grp = wp_repo.get_group_by_name(args.robot, _SEARCH_GROUP)
        existing_wp = (
            wp_repo.list_group_members(grp.id)
            if grp and grp.id is not None
            else []
        )
        if not existing_wp:
            raise SystemExit("기존 search 그룹 없음 — 관측 대역 baseline 불가")
        logger.info("기존 search 자세 %d개 특성화:", len(existing_wp))
        existing = [cam_pose_from_joints(kin, w.joint_values, w.name) for w in existing_wp]

        plan_ = plan(kin, existing, plane_z, inliers, intr, args.max_poses)
        base_pct = 100.0 * plan_.covered_existing.sum() / max(1, len(plan_.roi_cells))
        new_pct = 100.0 * plan_.covered_new.sum() / max(1, len(plan_.roi_cells))
        logger.info(
            "\n===== 결과 =====\n기존 %d자세 커버 %.0f%% → 제안 %d자세 커버 %.0f%%",
            len(existing), base_pct, len(plan_.chosen), new_pct,
        )
        render(plan_, existing, plane_z, Path(args.png))

        if args.commit:
            commit(
                wp_repo, args.robot, plan_.chosen, kin,
                group=args.group, backup=args.backup_group,
            )
        else:
            logger.info(
                "(dry-run — DB 미기록. --commit 으로 '%s' 기록"
                " + 원본은 '%s' 백업)", args.group, args.backup_group,
            )
    finally:
        kin.kinematics.close()


if __name__ == "__main__":
    main()
