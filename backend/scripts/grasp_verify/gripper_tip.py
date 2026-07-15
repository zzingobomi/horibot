"""손가락 mesh 의 실제 extent 를 tcp frame 으로 변환 → 진짜 손끝이 tcp 대비 어디 있나.
tcp x축 = approach. tcp 가 파지점이면 손끝(두 조가 만나는 지점)이 tcp 근처(x≈0)여야."""
import numpy as np
from yourdfpy import URDF

U = "../robot/so101_6dof/urdf/so101_6dof.urdf"
r = URDF.load(U, load_meshes=True)
r.update_cfg({"joint7": 0.0})  # 닫힘 근처


def tcp_of(link):
    return r.get_transform(frame_to=link, frame_from="tcp")


print("=== 손가락 mesh 정점의 tcp-frame extent (mm) ===")
print("tcp x축=approach(+가 물체쪽). 손끝이 tcp 사이면 x≈0 근처여야 함.\n")
for ln in ["gripper_back", "gripper_jaw"]:
    link = r.link_map.get(ln)
    if not link:
        print(f"{ln}: 없음"); continue
    T_link = tcp_of(ln)
    for v in (link.visuals or []):
        geom = v.geometry
        if not (geom.mesh and getattr(v, "geometry_mesh", None) is not None or geom.mesh):
            continue
        # yourdfpy: 로드된 trimesh 는 r.scene 에 있음 — 정점 직접 접근
    # scene 에서 이 링크 mesh 정점 가져오기
try:
    scene = r.scene
    for name, geom in scene.geometry.items():
        # geom.vertices 는 그 geometry 로컬. world transform 은 graph 에서.
        pass
except Exception as e:
    print("scene 접근 실패:", e)

# 더 단순: 각 링크의 collision/visual mesh 파일을 직접 trimesh 로드 후 링크→tcp 변환
import trimesh, os
MESH_DIR = "../robot/so101_6dof"
for ln in ["gripper_back", "gripper_jaw", "gripper_center"]:
    link = r.link_map.get(ln)
    if not link:
        continue
    T_link = tcp_of(ln)  # link → tcp
    for v in (link.visuals or []):
        g = v.geometry
        if not g.mesh:
            continue
        fn = g.mesh.filename
        path = os.path.join(MESH_DIR, fn) if not os.path.isabs(fn) else fn
        if not os.path.exists(path):
            # urdf workingpath 상대일 수 있음
            alt = os.path.join(MESH_DIR, "urdf", fn)
            path = alt if os.path.exists(alt) else path
        if not os.path.exists(path):
            print(f"{ln}: mesh 파일 못찾음 {fn}"); continue
        m = trimesh.load(path, force="mesh")
        vo = v.origin if v.origin is not None else np.eye(4)
        scale = np.array(g.mesh.scale) if g.mesh.scale is not None else np.ones(3)
        verts = m.vertices * scale
        # link local: origin 적용
        verts_link = (vo[:3, :3] @ verts.T).T + vo[:3, 3]
        # → tcp
        verts_tcp = (T_link[:3, :3] @ verts_link.T).T + T_link[:3, 3]
        mn = verts_tcp.min(0) * 1000
        mx = verts_tcp.max(0) * 1000
        print(f"{ln:14s} tcp-frame extent(mm): x[{mn[0]:6.1f},{mx[0]:6.1f}] "
              f"y[{mn[1]:6.1f},{mx[1]:6.1f}] z[{mn[2]:6.1f},{mx[2]:6.1f}]")
print("\n해석: 손끝(두 조가 맞물리는 앞쪽 끝)의 x_max 가 tcp(x=0)보다 한참 뒤(음수)면"
      " → tcp 가 손끝보다 물체쪽으로 튀어나옴 = 파지점 오정의.")
