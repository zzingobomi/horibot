"""URDF 그리퍼 기하: 손가락 링크/끝이 tcp 프레임 기준 어디 있나.
tcp 가 실제 파지점(손끝 사이)에 있어야 하는데 벗어나 있으면 = 파지 빗나감 + 바닥충돌 예측 실패."""
import numpy as np
from yourdfpy import URDF

U = "../robot/so101_6dof/urdf/so101_6dof.urdf"
r = URDF.load(U, load_meshes=False)
print("movable joints:", [j.name for j in r.robot.joints if j.type != "fixed"])
for j in r.robot.joints:
    if j.name == "joint7" and j.limit:
        print(f"joint7(gripper) limit: {j.limit.lower:.3f} ~ {j.limit.upper:.3f}")

def T(frame, base="tcp"):
    return r.get_transform(frame_to=frame, frame_from=base)

for cfg in [0.0]:
    r.update_cfg({"joint7": cfg})
    print(f"\n=== joint7={cfg} — 링크 원점 (tcp frame 기준, mm) ===")
    print("(tcp x축=approach 방향. tcp 가 손끝 사이면 손가락 링크가 tcp 근처여야)")
    for ln in ["gripper_fixed", "gripper_center", "gripper_back", "gripper_jaw", "tcp"]:
        try:
            t = T(ln)[:3, 3] * 1000
            print(f"  {ln:15s} = x={t[0]:7.1f}  y={t[1]:7.1f}  z={t[2]:7.1f}")
        except Exception as e:
            print(f"  {ln}: ERR {e}")

# 손가락 링크의 visual/collision geometry extent (링크 원점 기준) — 끝점 추정
print("\n=== 손가락 링크 geometry (링크 로컬, box/mesh) ===")
for ln in ["gripper_back", "gripper_jaw"]:
    link = r.link_map.get(ln)
    if link is None:
        print(f"  {ln}: 없음"); continue
    for v in (link.collisions or link.visuals or []):
        g = v.geometry
        og = v.origin[:3, 3] * 1000 if v.origin is not None else np.zeros(3)
        kind = "box" if g.box else ("mesh" if g.mesh else "other")
        size = (np.array(g.box.size) * 1000).tolist() if g.box else (g.mesh.filename if g.mesh else "?")
        print(f"  {ln}: {kind} origin(mm)=({og[0]:.1f},{og[1]:.1f},{og[2]:.1f}) size/mesh={size}")
