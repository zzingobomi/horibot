"""표면 antipodal 파지 선택 — 관측 점군에서 마주 보는 두 접촉점 쌍.

grasping.md §1: 윗면 footprint 파지(prismatic 전용 — 가려진
먼 면을 윗면 윤곽으로 **추측**) 폐기. 가정 없이 **관측된 표면**에서 조가 물 수
있는 antipodal 쌍(마주 보는 두 면, 법선 anti-parallel)을 찾는다. 단일 뷰는 마주
보는 면 중 먼 쪽이 항상 가려져 전 형상 0쌍 (§10.3-B) — 멀티뷰 융합 점군이 입력.

SO-101 은 조 축 수평 옆파지만 성립 → 조 축(두 접촉점을 잇는 선)이 수평인 쌍만.
파라미터는 §10.2 sim 전수 검증값 (scripts/grasp_verify 프로토타입 이관 —
box/원기둥/구/L자 concave × workspace 12위치 48/48 + 노이즈/bleed/clutter 통과).

open3d 법선 추정 — PC 전용 무거운 dep (scan build 와 같은 그룹)이라 import 는
사용 시점 (Pi/fast-test 의 import 비용 회피).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

Vec3 = tuple[float, float, float]

# 조 개구 한계 — 이보다 넓은 쌍은 물리적으로 못 물고, 좁은 쌍은 노이즈 자기쌍.
# 하한 8mm: 프로토타입 4mm 는 노이즈 σ1mm 의 4σ 거리라 단일 뷰에서도 edge 노이즈
# 가짜 쌍(w=4mm)이 생겼다 (2026-07-14 프로덕션 파이프라인 sim 재검증에서 발견).
# 스코프 물체(한 변 ~2cm)의 실 파지 폭 ≫ 8mm — 노이즈 플로어 위로 올려 차단.
_JAW_OPEN_MAX_M = 0.035
_JAW_WIDTH_MIN_M = 0.008
# 반대 접촉점 탐색: 접근선(−법선) 기준 측방 이탈 허용 + 법선 anti-parallel 허용각.
_LATERAL_TOL_M = 0.005
_ANTIPODAL_ANG_TOL_RAD = math.radians(25.0)
# 조 축 수평 판정 — 접근선의 z 성분 상한 (SO-101 옆파지 성립 조건).
_JAW_HORIZ_TOL_RAD = math.radians(20.0)
# 법선 추정 (voxel 다운샘플 후) — §10.2 검증 파라미터.
_VOXEL_M = 0.003
_NORMAL_RADIUS_M = 0.012
_NORMAL_MAX_NN = 30
_MAX_SEEDS = 300
# 유사 쌍 dedupe — 파지점/조 축이 사실상 같은 쌍은 후보 가족만 부풀린다.
_DEDUP_MID_M = 0.005
_DEDUP_AXIS_RAD = math.radians(15.0)


@dataclass(frozen=True, slots=True)
class AntipodalPair:
    """접촉쌍 1개 — mid: 두 접촉점 중점(파지점, base m), jaw_axis: 조 축
    (수평 단위 벡터, 접촉점 i→j 방향), width: 접촉점 간 거리 m."""

    mid: Vec3
    jaw_axis: Vec3
    width: float


def horizontal_antipodal_pairs(
    points: np.ndarray | list[Vec3], *, max_pairs: int = 12
) -> list[AntipodalPair]:
    """관측 점군 → 조 축 수평 antipodal 쌍 (파지점 중심이 점군 중심에 가까운
    순 — 물체 가장자리보다 몸통 중앙 파지를 선호). 점 부족/쌍 없음 = [].

    각 seed 점 p_i 의 접근선 d = −n_i 를 따라 폭 [MIN, MAX]·측방 ≤ LATERAL 안에
    반대 접촉점 p_j 를 찾고 n_j·d > cos(ANG_TOL)(anti-parallel) 이면 유효 쌍.
    법선은 centroid 바깥으로 orient — 볼록 몸통 기준 (L자 오목부도 §10.3 검증
    에서 쌍이 남았다: 바깥 면끼리의 쌍이 항상 존재).
    """
    import open3d as o3d  # PC 전용 무거운 dep — 사용 시점 로드

    pts_in = np.asarray(points, dtype=float)
    if pts_in.ndim != 2 or pts_in.shape[1] != 3 or len(pts_in) < 10:
        return []
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts_in)
    pc = pc.voxel_down_sample(_VOXEL_M)
    if len(pc.points) < 10:
        return []
    pc.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=_NORMAL_RADIUS_M, max_nn=_NORMAL_MAX_NN
        )
    )
    pts = np.asarray(pc.points)
    nrm = np.asarray(pc.normals)
    centroid = pts.mean(axis=0)
    outward = np.sum((pts - centroid) * nrm, axis=1) < 0
    nrm[outward] *= -1.0

    raw: list[tuple[np.ndarray, np.ndarray, float]] = []
    stride = max(1, len(pts) // _MAX_SEEDS)
    for i in range(0, len(pts), stride):
        d = -nrm[i]
        if abs(float(d[2])) > math.sin(_JAW_HORIZ_TOL_RAD):
            continue  # 접근선이 수평 아님 — 옆파지 조 축이 못 됨
        rel = pts - pts[i]
        t = rel @ d
        lat = np.linalg.norm(rel - np.outer(t, d), axis=1)
        cand = (t > _JAW_WIDTH_MIN_M) & (t < _JAW_OPEN_MAX_M) & (lat < _LATERAL_TOL_M)
        if not cand.any():
            continue
        aligned = (nrm[cand] @ d) > math.cos(_ANTIPODAL_ANG_TOL_RAD)
        if not aligned.any():
            continue
        j = int(np.where(cand)[0][aligned][np.argmin(lat[cand][aligned])])
        axis = pts[j] - pts[i]
        axis[2] = 0.0  # 조 축은 수평 성분만 (수평 옆파지)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-6:
            continue
        raw.append(((pts[i] + pts[j]) / 2.0, axis / norm, float(t[j])))

    # 중심 파지 선호 정렬 → 유사 쌍 dedupe → 상한
    raw.sort(key=lambda r: float(np.linalg.norm(r[0][:2] - centroid[:2])))
    out: list[AntipodalPair] = []
    for mid, axis, width in raw:
        dup = any(
            np.linalg.norm(mid - np.asarray(p.mid)) < _DEDUP_MID_M
            and abs(float(np.clip(axis @ np.asarray(p.jaw_axis), -1.0, 1.0)))
            > math.cos(_DEDUP_AXIS_RAD)
            for p in out
        )
        if dup:
            continue
        out.append(
            AntipodalPair(
                mid=(float(mid[0]), float(mid[1]), float(mid[2])),
                jaw_axis=(float(axis[0]), float(axis[1]), float(axis[2])),
                width=width,
            )
        )
        if len(out) >= max_pairs:
            break
    return out
