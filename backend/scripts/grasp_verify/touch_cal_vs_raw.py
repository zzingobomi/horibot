"""터치 관절각으로 '캘 켠 FK' vs '캘 끈 raw FK' 중 어느 쪽이 바닥을 더 평평하게 보나.
바닥은 물리적으로 평평(z=0) → 더 평평/일관되게 재구성하는 쪽이 옳은 모델.
- raw 가 더 평평 → 캘 보정이 오히려 재구성을 기울임 = 캘이 harmful (disable/redo)
- cal 이 더 평평 → 캘은 맞고 잔차는 hand_eye/touch noise
새 하드웨어 테스트 0 — 저장된 touch joints 만."""
import json, sqlite3
import numpy as np
from pathlib import Path
from scipy.optimize import least_squares
import sys
sys.path.insert(0, ".")
from apps.config import load_robots, _ROBOT_DIR
from modules.motor.contract import MotorKind
from modules.motion.fk_chain import FkChain
from modules.motion.urdf_patch import patch_urdf_link_offsets

ROBOT = "so101_6dof_0"
robot = load_robots()[ROBOT]
arm = [m for m in robot.motors if m.kind != MotorKind.GRIPPER]
arm_names = [m.name for m in arm]
n = len(arm)
urdf_raw = _ROBOT_DIR / robot.type / "urdf" / f"{robot.type}.urdf"

c = sqlite3.connect("horibot.db")
def load(kind):
    return json.loads(c.execute("SELECT result_data FROM calibration_results WHERE robot_id=? AND kind=? AND is_active=1", (ROBOT, kind)).fetchone()[0])
jo = load("joint_offset")["offsets"]; lo = load("link_offset")["offsets"]; sg = load("sag")["k_rad_per_m"]
id2i = {m.id: i for i, m in enumerate(arm)}
joint_off = np.array([jo.get(str(m.id), 0.0) for m in arm])
link_trans = np.zeros((n, 3)); link_rot = np.zeros((n, 3)); by_name = {}
for e in lo:
    i = id2i.get(e["joint_id"])
    if i is None: continue
    link_trans[i] = e["trans_m"]; link_rot[i] = e["rot_rad"]; by_name[arm[i].name] = (e["trans_m"], e["rot_rad"])
sag_idx = [id2i[int(k)] for k in sg if int(k) in id2i]
k_stiff = np.array([sg[str(arm[j].id)] for j in sag_idx]) if sag_idx else np.array([])

fk_raw = FkChain(urdf_raw, arm_names)
fk_pat = FkChain(patch_urdf_link_offsets(Path(urdf_raw), ROBOT, by_name), arm_names)

def R2q_none(R):  # not needed
    pass

caps = json.load(open("debug/touch_test_captures.json", encoding="utf-8"))["captures"]
saved_pos = np.array([c["position"] for c in caps])
saved_joints = np.array([c["joints"] for c in caps])  # = raw_to_rad + joint_offset

# --- 각 pose: cal FK (patched+sag, saved joints) / raw FK (raw urdf, joints - joint_offset) ---
def fk_cal(j):
    ja = fk_pat.apply_gravity_sag(j, k_stiff, sag_idx) if sag_idx else j
    return fk_pat.fk(ja)  # patched URDF 에 link_offset 이미 구움 — 재적용 X
def fk_raw_(j):
    return fk_raw.fk(j - joint_off)  # 캘 전부 제거

cal = [fk_cal(saved_joints[i]) for i in range(len(caps))]
raw = [fk_raw_(saved_joints[i]) for i in range(len(caps))]
cal_t = np.array([t for _, t in cal]); cal_R = [R for R, _ in cal]
raw_t = np.array([t for _, t in raw]); raw_R = [R for R, _ in raw]

# sanity: cal FK 가 저장된 runtime position 과 일치하나
err = np.linalg.norm(cal_t - saved_pos, axis=1) * 1000
print(f"[sanity] 재계산 cal FK vs 저장 runtime pos 오차(mm): max={err.max():.2f} mean={err.mean():.2f}")

def solve_tilt(ts, Rs):
    """tool 오프셋 f(±12cm) 물리 bound + 수직 시드로 평면 tilt/RMS."""
    def resid(x):
        th, ph, dd = x[:3]; f = x[3:6]
        nrm = np.array([np.sin(th)*np.cos(ph), np.sin(th)*np.sin(ph), np.cos(th)])
        return [nrm @ (ts[i] + Rs[i] @ f) - dd for i in range(len(ts))]
    best = None
    for seed in ([0,0,0],[0,0,-0.08],[0.05,0,-0.05],[0,0,0.05]):
        r = least_squares(resid, [0,0,ts[:,2].mean(),*seed],
                          bounds=([-0.6,-np.pi,-2,-0.12,-0.12,-0.12],[0.6,np.pi,2,0.12,0.12,0.12]))
        rms = np.sqrt(np.mean(np.array(resid(r.x))**2))*1000
        if best is None or rms < best[0]: best = (rms, r.x)
    rms, x = best
    return np.degrees(abs(x[0])), rms, x[3:6]

ct, cr, cf = solve_tilt(cal_t, cal_R)
rt, rr, rf = solve_tilt(raw_t, raw_R)
print("\n=== 캘 켬(cal) vs 캘 끔(raw) — 바닥 평면 ===")
print(f"  캘 켬 : tilt={ct:5.2f}°  RMS={cr:5.2f}mm  tool=({cf[0]*1000:.0f},{cf[1]*1000:.0f},{cf[2]*1000:.0f})")
print(f"  캘 끔 : tilt={rt:5.2f}°  RMS={rr:5.2f}mm  tool=({rf[0]*1000:.0f},{rf[1]*1000:.0f},{rf[2]*1000:.0f})")
print("\n판정:", end=" ")
if cr > 8 and rr > 8:
    print(f"양쪽 RMS 다 큼({cr:.0f}/{rr:.0f}mm) — 터치 noise 지배. tilt 대소만 참고.")
if rt + 1 < ct:
    print(f"raw 가 더 평평({rt:.1f}° < {ct:.1f}°) → 캘 보정이 바닥을 기울임 = 캘이 harmful 가능.")
elif ct + 1 < rt:
    print(f"cal 이 더 평평({ct:.1f}° < {rt:.1f}°) → 캘은 옳은 방향, 잔차는 hand_eye/noise.")
else:
    print(f"tilt 차이 미미({ct:.1f}° vs {rt:.1f}°) — 캘은 tilt 주범 아님. hand_eye 쪽.")
