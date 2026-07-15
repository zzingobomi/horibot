"""오늘 검출 프레임(깨끗한 depth)으로 hand_eye 보정량 δR,δt 를 직접 추정.
제약(둘 다 object-anywhere 안전 — 카메라 변환만 고침):
  (a) 각 프레임에서 테이블 평면이 평평(normal=+z) + z=0  → tilt/float 6자유도 중 3
  (b) 같은 큐브가 4개 뷰에서 같은 base 위치 (centroid 일치) → in-plane 3
보정 적용:  p_base' = R_be @ ( δR @ p_ee + δt ) + t_be,  p_ee = R_be^T (p_base - t_be)
결과 δR,δt → 새 hand_eye = δT ∘ (기존 hand_eye).  하드웨어 0.
"""
import json, glob, os
import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

SESS = "20260715_211723"
d = f"debug/detect/{SESS}"


def unproj(depth, scale, K, ys, xs):
    z = depth[ys, xs].astype(np.float64) * scale
    x = (xs - K["cx"]) / K["fx"] * z
    y = (ys - K["cy"]) / K["fy"] * z
    return np.stack([x, y, z], 1)


def base(cam, he, tcp):
    ee = cam @ np.array(he["R"]).T + np.array(he["t"])
    return ee @ np.array(tcp["R"]).T + np.array(tcp["t"])


frames = []
for jf in sorted(glob.glob(os.path.join(d, "*cube*.json"))):
    if "0003" in jf:  # 오검출
        continue
    m = json.load(open(jf, encoding="utf-8"))
    depth = cv2.imread(os.path.join(d, m["depth_png"]), cv2.IMREAD_UNCHANGED)
    K, scale = m["intrinsics"], m["depth_scale"]
    he, tcp = m["hand_eye_cam2ee"], m["tcp_ee2base"]
    R_be = np.array(tcp["R"]); t_be = np.array(tcp["t"])
    mask = cv2.imread(os.path.join(d, m["candidates"][0]["mask_png"]), cv2.IMREAD_UNCHANGED) > 0
    valid = depth > 0
    # 큐브 점 (base, 현재 변환)
    cy, cx = np.nonzero(mask & valid)
    idx = np.linspace(0, len(cy)-1, min(len(cy), 400)).astype(int)
    cube_base = base(unproj(depth, scale, K, cy[idx], cx[idx]), he, tcp)
    # 테이블 ring (base)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (81, 81))
    klo = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
    ring = (cv2.dilate(mask.astype(np.uint8), k) > 0) & ~(cv2.dilate(mask.astype(np.uint8), klo) > 0) & valid
    ty, tx = np.nonzero(ring)
    idx2 = np.linspace(0, len(ty)-1, min(len(ty), 600)).astype(int)
    tbl_base = base(unproj(depth, scale, K, ty[idx2], tx[idx2]), he, tcp)
    tbl_base = tbl_base[np.abs(tbl_base[:,2]-np.median(tbl_base[:,2]))<0.02]
    # base → ee (보정 적용 준비)
    def to_ee(pb):
        return (pb - t_be) @ R_be
    frames.append(dict(R_be=R_be, t_be=t_be, cube_ee=to_ee(cube_base), tbl_ee=to_ee(tbl_base)))

print(f"프레임 {len(frames)}개")


def apply(fr, dR, dt):
    """보정 후 base 점."""
    def corr(p_ee):
        return (p_ee @ dR.T + dt) @ fr["R_be"].T + fr["t_be"]
    return corr(fr["cube_ee"]), corr(fr["tbl_ee"])


def resid(x):
    dR = Rot.from_rotvec(x[:3]).as_matrix()
    dt = x[3:6]
    cube_cens = []
    out = []
    for fr in frames:
        cube_b, tbl_b = apply(fr, dR, dt)
        # 테이블 평면: normal → +z, z → 0
        c = tbl_b.mean(0)
        _, _, vt = np.linalg.svd(tbl_b - c)
        n = vt[2]; n = n if n[2] > 0 else -n
        out += [n[0]*3, n[1]*3, c[2]*20.0]   # tilt(강조) + z(강조)
        cube_cens.append(cube_b.mean(0))
    cube_cens = np.array(cube_cens)
    mean_c = cube_cens.mean(0)
    for cc in cube_cens:
        out += list((cc - mean_c) * 10.0)    # 큐브 위치 일관성 (in-plane 구속)
    return out


# 보정 전 상태
def report(x, tag):
    dR = Rot.from_rotvec(x[:3]); dt = x[3:6]
    tilts, tblz, cens = [], [], []
    for fr in frames:
        cb, tb = apply(fr, dR.as_matrix(), dt)
        c = tb.mean(0); _,_,vt = np.linalg.svd(tb-c); n=vt[2]; n=n if n[2]>0 else -n
        tilts.append(np.degrees(np.arccos(min(abs(n[2]),1))))
        tblz.append(c[2]*1000); cens.append(cb.mean(0))
    cens = np.array(cens)
    print(f"\n[{tag}] 테이블 기울기 {np.round(tilts,1)}° / 테이블z {np.round(tblz,1)}mm")
    print(f"        큐브centroid 산포(mm): {np.round(cens.std(0)*1000,1)} (작을수록 뷰간 일치)")


report(np.zeros(6), "보정 전 (현재)")
res = least_squares(resid, np.zeros(6), method="lm")
report(res.x, "보정 후")
dR = Rot.from_rotvec(res.x[:3]); dt = res.x[3:6]
print(f"\n=== 추정 hand_eye 보정 δT ===")
print(f"  δR = {dR.magnitude()*180/np.pi:.2f}°  (euler xyz° = {np.round(dR.as_euler('xyz',degrees=True),2)})")
print(f"  δt = ({dt[0]*1000:.1f}, {dt[1]*1000:.1f}, {dt[2]*1000:.1f}) mm")

# 새 hand_eye = δT ∘ 기존.  p_ee' = δR p_ee + δt, 기존 p_ee = R_he p_cam + t_he
he0 = frames and json.load(open(sorted(glob.glob(os.path.join(d,'*cube*.json')))[0],encoding='utf-8'))["hand_eye_cam2ee"]
R_he = np.array(he0["R"]); t_he = np.array(he0["t"])
R_new = dR.as_matrix() @ R_he
t_new = dR.as_matrix() @ t_he + dt
print(f"\n=== 적용할 새 hand_eye (cam2gripper) ===")
print("R_new =", np.array2string(R_new, precision=6))
print("t_new =", np.array2string(t_new, precision=6))
np.set_printoptions(suppress=True)
json.dump({"R_cam2gripper": R_new.tolist(), "t_cam2gripper": t_new.reshape(3,1).tolist(),
           "delta_R_deg": dR.magnitude()*180/np.pi, "delta_t_mm": (dt*1000).tolist()},
          open("debug/handeye_correction.json","w"), indent=1)
print("\n[저장] debug/handeye_correction.json")
