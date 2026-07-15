"""debug PLY 점군 정량 분석 — z 분포/extent/centroid, 단일뷰 vs 융합."""
import sys
import glob
import os
import numpy as np
import open3d as o3d


def stats(name, xyz):
    z = xyz[:, 2]
    mn = xyz.min(axis=0)
    mx = xyz.max(axis=0)
    ext = (mx - mn) * 1000  # mm
    cen = xyz.mean(axis=0)
    # top-band centroid (윗면 25%내 band) — geometry.top_face_points 재현
    top_ref = np.percentile(z, 75.0)
    topmask = z >= top_ref - 0.010
    tcen = xyz[topmask].mean(axis=0)
    p98 = np.percentile(z, 98.0)
    print(f"{name:36s} N={len(xyz):5d} "
          f"ext(mm) X={ext[0]:5.1f} Y={ext[1]:5.1f} Z={ext[2]:5.1f} "
          f"z[min..max]={z.min()*1000:6.1f}..{z.max()*1000:6.1f} "
          f"z(p2,p50,p98)={np.percentile(z,2)*1000:.0f},"
          f"{np.percentile(z,50)*1000:.0f},{p98*1000:.0f} "
          f"cen=({cen[0]*1000:.0f},{cen[1]*1000:.0f},{cen[2]*1000:.0f}) "
          f"topcen=({tcen[0]*1000:.0f},{tcen[1]*1000:.0f},{tcen[2]*1000:.0f})")
    hist, edges = np.histogram(z * 1000, bins=10)
    binlbl = " ".join(f"{h:3d}" for h in hist)
    print(f"{'':36s} z-hist[{edges[0]:.0f}..{edges[-1]:.0f}mm /10]: {binlbl}")


session = sys.argv[1] if len(sys.argv) > 1 else "20260714_233959"
d = f"debug/detect/{session}"
files = sorted(glob.glob(os.path.join(d, "*.ply")))
print(f"=== {session} : {len(files)} PLY ===\n")
for f in files:
    name = os.path.basename(f).replace("white_small_round_cube", "cube")
    pc = o3d.io.read_point_cloud(f)
    xyz = np.asarray(pc.points)
    if len(xyz) == 0:
        print(f"{name}: EMPTY")
        continue
    stats(name, xyz)
    print()
