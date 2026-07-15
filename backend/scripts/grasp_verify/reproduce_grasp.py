"""실 PLY 점군에 실제 모듈 함수를 그대로 돌려 로봇이 계산한 파지점을 재현.
하드웨어 0 — 어제 저장된 base-frame 점군이 입력. z 구조를 정밀 분해한다."""
import sys
import glob
import os
import numpy as np
import open3d as o3d

sys.path.insert(0, "modules")
sys.path.insert(0, ".")
from modules.detector import geometry
from modules.tasks.pick_and_place import antipodal


def zclusters(z, gap=0.005):
    """z 를 gap(5mm) 이상 빈틈으로 군집 → 각 군집 (zmin,zmax,count)."""
    zs = np.sort(z)
    cuts = np.nonzero(np.diff(zs) > gap)[0]
    bounds = [0, *(cuts + 1), len(zs)]
    out = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        seg = zs[a:b]
        out.append((seg.min() * 1000, seg.max() * 1000, len(seg)))
    return out


def analyze(name, xyz):
    z = xyz[:, 2]
    print(f"\n### {name}  N={len(xyz)}")
    # z 군집 분해 (유령 점 vs 몸통)
    cl = zclusters(z)
    print("  z-clusters(mm, gap5mm):", [f"{a:.0f}-{b:.0f}(n{n})" for a,b,n in cl])
    # object_metrics (로봇이 쓴 그대로)
    m = geometry.object_metrics_from_points(xyz)
    if m:
        pos, base_z, h = m
        print(f"  object_metrics: pos(top)=({pos[0]*1000:.0f},{pos[1]*1000:.0f},"
              f"{pos[2]*1000:.0f})mm base_z={base_z*1000:.0f}mm height={h*1000:.0f}mm")
    # top-face band centroid xy (파지 yaw/footprint 근거)
    tf = geometry.top_face_points(xyz)
    if tf is not None:
        tc = tf.mean(axis=0)
        print(f"  top_face band: N={len(tf)} cen=({tc[0]*1000:.0f},"
              f"{tc[1]*1000:.0f},{tc[2]*1000:.0f})mm zspan="
              f"{tf[:,2].min()*1000:.0f}..{tf[:,2].max()*1000:.0f}")
    # 실제 antipodal 파지쌍 (로봇이 채택하는 후보들)
    try:
        pairs = antipodal.horizontal_antipodal_pairs(xyz, max_pairs=6)
        if pairs:
            print(f"  antipodal pairs: {len(pairs)}")
            for i, p in enumerate(pairs[:4]):
                print(f"    pair{i}: mid=({p.mid[0]*1000:.0f},{p.mid[1]*1000:.0f},"
                      f"{p.mid[2]*1000:.0f})mm  w={p.width*1000:.0f}mm  "
                      f"axis=({p.jaw_axis[0]:.2f},{p.jaw_axis[1]:.2f},{p.jaw_axis[2]:.2f})")
        else:
            print("  antipodal pairs: 0")
    except Exception as e:
        print(f"  antipodal ERR: {e}")


session = sys.argv[1] if len(sys.argv) > 1 else "20260714_233959"
d = f"debug/detect/{session}"
# 큐브 단일뷰 + 융합만
for f in sorted(glob.glob(os.path.join(d, "*cube*.ply"))):
    pc = o3d.io.read_point_cloud(f)
    xyz = np.asarray(pc.points)
    # garbage(테이블 전체 잡은 것) 제외 — footprint 10cm 넘으면 skip
    if len(xyz) < 5:
        continue
    ext = (xyz.max(0) - xyz.min(0)) * 1000
    if ext[0] > 100 or ext[1] > 100:
        continue
    analyze(os.path.basename(f).replace("white_small_round_cube","cube"), xyz)
