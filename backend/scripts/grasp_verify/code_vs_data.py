"""코드 버그 vs depth 편향을 가르는 결정적 테스트.

3 케이스에 **프로덕션 함수 그대로**(detector.geometry + pick_and_place):
  A. 실 융합 점군(0011_fuse, 편향됨) → 파지 z (baseline 재현)
  B. 실 점군을 바닥이 z=0 에 오게 하강(편향만 제거) → 파지 z (코드 정상이면 큐브 안)
  C. 완전 합성 클린 큐브(참값 0~25mm, 편향 0) → 파지 z (코드 버그면 여기서도 높음)
"""
import numpy as np
import open3d as o3d

from modules.detector import geometry as dg
from modules.tasks.pick_and_place import antipodal, geometry as pg


def grasp_z_summary(tag, xyz, true_center_z_mm=12.5, cube_top_mm=25.0):
    m = dg.object_metrics_from_points(xyz)
    pos, base_z, h = m
    pairs = antipodal.horizontal_antipodal_pairs(xyz, max_pairs=12)
    if not pairs:
        print(f"[{tag}] antipodal 0쌍")
        return
    mids = np.array([p.mid for p in pairs]) * 1000
    widths = np.array([p.width for p in pairs]) * 1000
    plan = pg.plan_grasp(pairs)
    gz = np.array([c.grasp[2] for c in plan]) * 1000  # 실행되는 파지 z (mid+lateral)
    print(f"\n[{tag}]")
    print(f"  점군 z범위: {xyz[:,2].min()*1000:.0f}..{xyz[:,2].max()*1000:.0f}mm "
          f" object_metrics base_z={base_z*1000:.0f} top={pos[2]*1000:.0f} h={h*1000:.0f}mm")
    print(f"  antipodal {len(pairs)}쌍  mid_z={mids[:,2].min():.0f}..{mids[:,2].max():.0f}mm"
          f"  w={widths.min():.0f}..{widths.max():.0f}mm")
    print(f"  plan_grasp {len(plan)}후보  실행 grasp_z={gz.min():.0f}..{gz.max():.0f}mm")
    # 판정: 파지 z 가 큐브 몸통(0~top) 안이면 코드 정상
    inside = ((gz >= -2) & (gz <= cube_top_mm + 2)).mean() * 100
    print(f"  → 참 큐브(0~{cube_top_mm:.0f}mm) 안에 드는 후보 비율: {inside:.0f}%  "
          f"(중심 {true_center_z_mm}mm 기준)")


def synth_cube_surface(cx, cy, side_m=0.025, base_z=0.0, n=1200, noise=0.0005, seed=0):
    """책상 위 정육면체의 관측 가능 표면(윗면 + 옆면 4개) 점 샘플 — 클린 depth 가정.
    바닥면은 책상에 닿아 안 보임(가림) = 실 관측과 동일. 편향 0."""
    rng = np.random.default_rng(seed)
    s = side_m
    hz = base_z
    pts = []
    # 윗면 z=base+side
    top = rng.uniform([cx - s/2, cy - s/2, hz + s], [cx + s/2, cy + s/2, hz + s], (n//5, 3))
    pts.append(top)
    # 옆면 4개 (z: base~base+side)
    for ax, val in [(0, cx - s/2), (0, cx + s/2), (1, cy - s/2), (1, cy + s/2)]:
        f = rng.uniform([cx - s/2, cy - s/2, hz], [cx + s/2, cy + s/2, hz + s], (n//5, 3))
        f[:, ax] = val
        pts.append(f)
    out = np.vstack(pts)
    out += rng.normal(0, noise, out.shape)
    return out


# A. 실 융합 점군 (편향됨)
real = np.asarray(o3d.io.read_point_cloud(
    "debug/detect/20260714_233959/0011_fuse_white_small_round_cube_c0.ply").points)
grasp_z_summary("A 실데이터(편향)", real)

# B. 편향만 제거 (바닥 z=0 으로 하강)
shifted = real.copy()
shifted[:, 2] -= real[:, 2].min()  # 최저점을 0 으로
grasp_z_summary("B 실데이터-편향제거(바닥→0)", shifted)

# C. 완전 합성 클린 큐브 (편향 0, 참값 0~25mm)
synth = synth_cube_surface(0.27, 0.125, side_m=0.025, base_z=0.0)
grasp_z_summary("C 합성 클린 큐브(편향0)", synth)
