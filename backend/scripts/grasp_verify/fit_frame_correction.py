"""결정적 회귀 진단: 오늘 4프레임의 테이블이 전부 사선/부양인데,
EE 프레임의 '고정' 보정 하나(δR 3 + δt 3)로 전 프레임 테이블을 동시에
평평(normal=+z) + z=0 으로 만들 수 있는가?

- 가능(잔차 작음) → 오차는 EE 프레임의 '고정' transform (hand_eye R/t 또는 TCP 링크
  프레임이 캘 당시와 어긋남). v1↔v2 회귀와 정합. 보정량 = 얼마나 어긋났나.
- 불가(프레임마다 제각각) → 고정 프레임 오차 아님 (FK 자세의존 오차 등).

모델: p_ee = R_ce·p_cam + t_ce (저장 hand_eye) → 보정 p_ee' = δR·p_ee + δt
      p_base = R_be·p_ee' + t_be
하드웨어 0 — 오늘 debug 데이터만."""
import sys, os, json, glob
import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot


def load_frames(session):
    d = f"debug/detect/{session}"
    frames = []
    for jf in sorted(glob.glob(os.path.join(d, "*cube*.json"))):
        if "0003" in jf:
            continue
        meta = json.load(open(jf, encoding="utf-8"))
        depth = cv2.imread(os.path.join(d, meta["depth_png"]), cv2.IMREAD_UNCHANGED)
        K = meta["intrinsics"]; scale = meta["depth_scale"]
        mask = cv2.imread(os.path.join(d, meta["candidates"][0]["mask_png"]),
                          cv2.IMREAD_UNCHANGED) > 0
        valid = depth > 0
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (81, 81))
        klo = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41))
        ring = (cv2.dilate(mask.astype(np.uint8), k) > 0) & \
               ~(cv2.dilate(mask.astype(np.uint8), klo) > 0) & valid
        ys, xs = np.nonzero(ring)
        # 다운샘플
        idx = np.linspace(0, len(ys) - 1, min(len(ys), 1500)).astype(int)
        ys, xs = ys[idx], xs[idx]
        z = depth[ys, xs].astype(np.float64) * scale
        x = (xs - K["cx"]) / K["fx"] * z
        y = (ys - K["cy"]) / K["fy"] * z
        p_cam = np.stack([x, y, z], 1)
        R_ce = np.array(meta["hand_eye_cam2ee"]["R"]); t_ce = np.array(meta["hand_eye_cam2ee"]["t"])
        R_be = np.array(meta["tcp_ee2base"]["R"]); t_be = np.array(meta["tcp_ee2base"]["t"])
        p_ee = p_cam @ R_ce.T + t_ce
        frames.append((os.path.basename(jf), p_ee, R_be, t_be))
    return frames


def plane_resid(pts):
    """평면 피팅 → (tilt_from_z 잔차벡터, z@centroid)."""
    c = pts.mean(0)
    _, _, vt = np.linalg.svd(pts - c)
    n = vt[2]
    if n[2] < 0:
        n = -n
    # 목표: n = (0,0,1), c_z = 0
    return np.array([n[0], n[1], c[2] * 30.0])  # z 를 가중(미터→강조)


def residuals(x, frames):
    dR = Rot.from_rotvec(x[:3]).as_matrix()
    dt = x[3:6]
    out = []
    for _, p_ee, R_be, t_be in frames:
        p_ee2 = p_ee @ dR.T + dt
        p_base = p_ee2 @ R_be.T + t_be
        # robust: 중앙값 주변만
        zc = p_base[:, 2]
        keep = np.abs(zc - np.median(zc)) < 0.02
        out.append(plane_resid(p_base[keep]))
    return np.concatenate(out)


session = sys.argv[1] if len(sys.argv) > 1 else "20260715_211723"
frames = load_frames(session)
print(f"프레임 {len(frames)}개\n")

print("=== 보정 전 (현재 v2 상태) ===")
for name, p_ee, R_be, t_be in frames:
    pb = p_ee @ R_be.T + t_be
    keep = np.abs(pb[:, 2] - np.median(pb[:, 2])) < 0.02
    r = plane_resid(pb[keep])
    tilt = np.degrees(np.arctan2(np.hypot(r[0], r[1]), 1))
    print(f"  {name}: 기울기={tilt:4.1f}°  z@중심={r[2]/30*1000:+5.1f}mm")

res = least_squares(residuals, np.zeros(6), args=(frames,), method="lm")
dR = Rot.from_rotvec(res.x[:3])
dt = res.x[3:6]
print(f"\n=== 고정 EE 보정 fit (δR, δt 6dof) ===")
print(f"  δR = {dR.magnitude()*180/np.pi:.2f}° axis={dR.as_rotvec()/ (dR.magnitude()+1e-12)}")
print(f"  δR euler(xyz deg) = {dR.as_euler('xyz', degrees=True)}")
print(f"  δt = {dt*1000} mm")
print(f"\n=== 보정 후 ===")
for name, p_ee, R_be, t_be in frames:
    p_ee2 = p_ee @ dR.as_matrix().T + dt
    pb = p_ee2 @ R_be.T + t_be
    keep = np.abs(pb[:, 2] - np.median(pb[:, 2])) < 0.02
    r = plane_resid(pb[keep])
    tilt = np.degrees(np.arctan2(np.hypot(r[0], r[1]), 1))
    print(f"  {name}: 기울기={tilt:4.1f}°  z@중심={r[2]/30*1000:+5.1f}mm")
