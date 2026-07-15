"""정지한 큐브가 뷰마다 다른 위치로 찍히는 원인을 직접 규명.
각 뷰: 큐브 base 위치(top/센터) + 그 순간 카메라 위치/시선(ee pose) + depth 품질.
- 큐브 추정이 카메라 위치 따라 움직이면 → FK/hand_eye (기구학) 오차
- depth valid 낮은 뷰만 튀면 → depth 품질
하드웨어 0."""
import json, glob, os
import numpy as np
import cv2

S = "20260715_234827"
d = f"debug/detect/{S}"


def unproj(depth, scale, K, ys, xs):
    z = depth[ys, xs].astype(np.float64) * scale
    x = (xs - K["cx"]) / K["fx"] * z
    y = (ys - K["cy"]) / K["fy"] * z
    return np.stack([x, y, z], 1)


rows = []
for jf in sorted(glob.glob(os.path.join(d, "*cube*.json"))):
    if "0003" in jf:
        continue
    m = json.load(open(jf, encoding="utf-8"))
    depth = cv2.imread(os.path.join(d, m["depth_png"]), cv2.IMREAD_UNCHANGED)
    K, scale = m["intrinsics"], m["depth_scale"]
    he, tcp = m["hand_eye_cam2ee"], m["tcp_ee2base"]
    R_be = np.array(tcp["R"]); t_be = np.array(tcp["t"])
    R_ce = np.array(he["R"]); t_ce = np.array(he["t"])
    mask = cv2.imread(os.path.join(d, m["candidates"][0]["mask_png"]), cv2.IMREAD_UNCHANGED) > 0
    valid = depth > 0
    area = int(mask.sum()); nvalid = int((mask & valid).sum())
    ys, xs = np.nonzero(mask & valid)
    cam = unproj(depth, scale, K, ys, xs)
    base = (cam @ R_ce.T + t_ce) @ R_be.T + t_be
    # robust 센터/top
    z = base[:, 2]
    top = np.percentile(z, 98) * 1000
    botn = np.percentile(z, 2) * 1000
    cen = np.median(base, 0) * 1000
    # 카메라 위치(base) = ee 에 hand_eye t 실은 것
    cam_pos = (R_be @ t_ce + t_be) * 1000
    # 카메라 시선(base) = R_be R_ce [0,0,1]
    view = R_be @ R_ce @ np.array([0, 0, 1.0])
    rows.append((os.path.basename(jf)[:4], cen, top, botn, cam_pos, view,
                 100 * nvalid / max(area, 1), len(ys)))

print(f"=== {S}: 정지 큐브 뷰별 (전부 같은 물체) ===")
print(f"{'뷰':>5} {'큐브센터(base mm)':>22} {'top':>5} {'bot':>5} | "
      f"{'카메라위치(base mm)':>22} {'valid%':>6} {'N':>4}")
cens = []
for name, cen, top, botn, cpos, view, vf, n in rows:
    cens.append(cen)
    print(f"{name:>5} ({cen[0]:6.0f},{cen[1]:6.0f},{cen[2]:6.0f})  {top:5.0f} {botn:5.0f} | "
          f"({cpos[0]:6.0f},{cpos[1]:6.0f},{cpos[2]:6.0f}) {vf:6.1f} {n:4d}")
cens = np.array(cens)
print(f"\n큐브 센터 산포(std, mm): x={cens[:,0].std():.1f} y={cens[:,1].std():.1f} z={cens[:,2].std():.1f}")
print(f"큐브 센터 범위(mm): x={cens[:,0].ptp():.0f} y={cens[:,1].ptp():.0f} z={cens[:,2].ptp():.0f}")
print("→ 산포가 크면(cm급) 정지 물체가 뷰마다 다르게 보이는 것 = 캘/FK 오차 확정.")
