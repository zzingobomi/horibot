"""파이프라인 수정 전 검증: SOR(flying-pixel 제거)이 실제로 뷰별 큐브 extent 를
25mm 로 줄이고, fuse 덩어리도 깨끗해지는지 실 debug 데이터로 확인.
되면 detector 파이프라인에 넣는다. 하드웨어 0."""
import json, glob, os
import numpy as np
import cv2
import open3d as o3d
import sys
sys.path.insert(0, ".")
from modules.detector import projection, geometry

S = "20260715_234827"
d = f"debug/detect/{S}"


def sor(pts, nb=16, std=2.0):
    if pts is None or len(pts) < nb:
        return pts
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts)
    pc, _ = pc.remove_statistical_outlier(nb_neighbors=nb, std_ratio=std)
    return np.asarray(pc.points)


def ext(pts):
    return (pts.max(0) - pts.min(0)) * 1000 if pts is not None and len(pts) else np.zeros(3)


clean_views = []
print(f"=== {S}: SOR 정리 전/후 (close 뷰) ===")
print(f"{'뷰':>5} {'before X/Y/Z mm':>20} {'after X/Y/Z mm':>20} {'N전':>5} {'N후':>5}")
for jf in sorted(glob.glob(os.path.join(d, "*cube*.json"))):
    if "0003" in jf:
        continue
    m = json.load(open(jf, encoding="utf-8"))
    depth = cv2.imread(os.path.join(d, m["depth_png"]), cv2.IMREAD_UNCHANGED)
    K, scale = m["intrinsics"], m["depth_scale"]
    he, tcp = m["hand_eye_cam2ee"], m["tcp_ee2base"]
    mask = cv2.imread(os.path.join(d, m["candidates"][0]["mask_png"]), cv2.IMREAD_UNCHANGED) > 0
    pts = projection.base_points_from_mask(
        depth, mask, scale, K["fx"], K["fy"], K["cx"], K["cy"],
        np.array(tcp["R"]), np.array(tcp["t"]), np.array(he["R"]), np.array(he["t"]),
    )
    if pts is None:
        continue
    e0 = ext(pts)
    cl = sor(pts)
    e1 = ext(cl)
    name = os.path.basename(jf)[:4]
    print(f"{name:>5} {e0[0]:5.0f}/{e0[1]:4.0f}/{e0[2]:4.0f}        "
          f"{e1[0]:5.0f}/{e1[1]:4.0f}/{e1[2]:4.0f}        {len(pts):5d} {len(cl):5d}")
    # close 뷰만(멀리 search 뷰 0001,0002 제외) fuse 검증용 수집
    if name not in ("0001", "0002"):
        clean_views.append((cl, tuple(np.median(cl, 0))))

# fuse 검증: 정리된 close 뷰들 병합
if len(clean_views) >= 2:
    merged = geometry.align_and_merge_views(
        [v for v, _ in clean_views], [p for _, p in clean_views]
    )
    em = ext(merged)
    mt = geometry.object_metrics_from_points(merged)
    print(f"\n=== 정리된 close 뷰 {len(clean_views)}개 fuse 결과 ===")
    print(f"  fuse extent(mm): X={em[0]:.0f} Y={em[1]:.0f} Z={em[2]:.0f}  (큐브 25mm 목표)")
    if mt:
        pos, bz, h = mt
        print(f"  object_metrics: base_z={bz*1000:.0f}mm height={h*1000:.0f}mm pos=({pos[0]*1000:.0f},{pos[1]*1000:.0f})")
    print("  (정리 안 했을 때 fuse 는 49x56x41mm 덩어리였음)")
