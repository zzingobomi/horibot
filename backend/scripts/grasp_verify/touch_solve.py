"""터치 데이터에서 tool 오프셋 + 진짜 바닥 평면 동시 추정 (자세 varied 보정).
접촉점 c_i = p_i(TCP) + R_i @ t.  모든 c_i 는 한 평면 n·x=d 위.
degeneracy 방지: 평면을 (θ,φ,d) 로, tool t 는 물리 범위(±20cm) bound + 수직 시드.
하드웨어 0."""
import json
import numpy as np
from scipy.optimize import least_squares

d = json.load(open("debug/touch_test_captures.json", encoding="utf-8"))
caps = d["captures"]
P = np.array([c["position"] for c in caps])
Q = np.array([c["quaternion"] for c in caps])


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


Rs = [quat_R(q) for q in Q]


def nrm(theta, phi):
    return np.array([np.sin(theta)*np.cos(phi), np.sin(theta)*np.sin(phi), np.cos(theta)])


def resid(x):
    theta, phi, dd, tx, ty, tz = x
    n = nrm(theta, phi)
    t = np.array([tx, ty, tz])
    out = []
    for i in range(len(P)):
        c = P[i] + Rs[i] @ t
        out.append(n @ c - dd)
    return out


z0 = P[:, 2].mean()
seed = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
lo = [-0.6, -np.pi, -2.0, -0.2, -0.2, -0.2]
hi = [0.6, np.pi, 2.0, 0.2, 0.2, 0.2]
res = least_squares(resid, seed, bounds=(lo, hi), method="trf")
theta, phi, dd, tx, ty, tz = res.x
n = nrm(theta, phi); t = np.array([tx, ty, tz])
contacts = np.array([P[i] + Rs[i] @ t for i in range(len(P))])
tilt = np.degrees(theta)
rms = np.sqrt(np.mean(np.array(resid(res.x))**2)) * 1000
cz = contacts[:, 2] * 1000

print("=== tool 오프셋 + 바닥 평면 (bound+수직시드) ===")
print(f"tool 오프셋(TCP frame,mm): ({t[0]*1000:.1f},{t[1]*1000:.1f},{t[2]*1000:.1f}) |t|={np.linalg.norm(t)*1000:.1f}mm")
print(f"평면 RMS 잔차: {rms:.2f}mm")
print(f"법선=({n[0]:+.3f},{n[1]:+.3f},{n[2]:+.3f})")
print(f"★ FK-바닥 기울기 = {tilt:.2f}°")
print(f"접촉점(=바닥) base z(mm): min={cz.min():.1f} max={cz.max():.1f} mean={cz.mean():.1f}")

# 참고: 자세 거의 수직인 점만으로 독립 교차검증
Qref = Q[np.argmin(np.abs(Q[:, 3]))]  # 임의 기준
ang_to = np.degrees(2*np.arccos(np.clip(np.abs(Q @ Q[0]), 0, 1)))
print(f"\n(자세 편차 분포°: {np.array2string(ang_to, precision=0)})")

print("\n판정:", end=" ")
if rms > 5:
    print(f"RMS {rms:.1f}mm 큼 — 풀이 신뢰 낮음. tool bound 걸림 여부 확인 필요.")
elif tilt < 2:
    print(f"기울기 {tilt:.1f}° → FK 바닥 평평 = FK 정확. 재구성 사선은 hand_eye 탓.")
elif tilt < 8:
    print(f"기울기 {tilt:.1f}° → FK 자체가 기울어 봄 = 기구학/base 기여.")
else:
    print(f"기울기 {tilt:.1f}° 큼.")
