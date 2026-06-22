"""원본 URDF 에 link_offsets 를 patch 적용한 URDF text 를 in-memory 반환.

설계 (docs/storage_layer.md §13):
    SSOT 는 storage DB 의 link_offset row. URDF 는 부팅 시점에 render 되는
    파생물 (디스크 영속화 X). PyBullet `loadURDF` 가 path-only API 라
    caller (PybulletKinematics.initialize) 가 tempfile 로 1회성 우회.

    산업 표준 패턴 — ROS 2 `robot_description` parameter (URDF *string* 으로 들고
    다님, 파일 아님) / UR `ur_calibration` (YAML SSOT + xacro substitute) /
    Franka `franka_description` (kinematics.yaml + xacro) 와 동형.

URDF rpy 가산 — small-angle 가정:
    BA가 추정하는 link_rot은 axis-angle rotation vector (rad). URDF의
    <joint><origin rpy="..."/>는 ZYX 오일러. 일반적으로 두 표현이 다르나, 작은
    각(<5°)에서는 rpy ≈ rotvec 근사 정확. 현재 v3 final 결과 link_rot 최대
    0.85° 수준이라 안전. 큰 각이 필요해지면 정확한 matrix→rpy 변환 도입
    (Option B / xacro 도입 시점에 같이 정리).
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from modules.calibration.result_models import LinkOffsetResultData

logger = logging.getLogger(__name__)


def _parse_xyz(s: str) -> np.ndarray:
    parts = s.strip().split()
    return np.array([float(p) for p in parts[:3]], dtype=np.float64)


def _fmt_xyz(v: np.ndarray) -> str:
    return f"{v[0]:.9g} {v[1]:.9g} {v[2]:.9g}"


def patch_urdf_text(
    source_urdf_path: str | Path,
    offsets: LinkOffsetResultData,
    joint_id_map: dict[str, int] | None = None,
) -> str:
    """원본 URDF 파일을 읽어 link_offsets patch 적용한 URDF 텍스트 반환.

    부수 변경: <mesh filename="...">의 상대경로를 절대경로로 rewrite (tempfile load
    시점에 원본 mesh 경로 lazy resolve 위해, storage_layer.md §13.5).

    multi-robot 컨벤션 — URDF joint naming = `joint{N}` (1-indexed, arm DOF 만큼).
    `joint_id_map` 미주입 시 LinkOffsetResultData 의 entry id 가 SSOT — omx_f (5DOF)
    / so101_6dof (6DOF) 등 robot type 무관 자동 동작. tcp_joint / 그리퍼 joint 는
    offsets entry 에 들어가지 않으므로 자동 제외 (tcp_joint 는 캘리브레이션 reference
    frame 으로 고정 — 한때 patch 시도했으나 detect↔IK self-consistency 로 cancel
    out 됨, 2026-05-28 실측 확인).

    Args:
        source_urdf_path: 원본 .urdf 경로.
        offsets: LinkOffsetResultData — joint id별 (trans_m, rot_rad rotvec) entry list.
        joint_id_map: URDF joint name → motor id 매핑. None 이면 offsets entry 에서
            `joint{N}` 컨벤션으로 자동 도출.

    Returns:
        patched URDF text (utf-8 encoding 가능).
    """
    if joint_id_map is None:
        ids = sorted(e.joint_id for e in offsets.offsets)
        joint_id_map = {f"joint{i}": i for i in ids}

    src = Path(source_urdf_path)
    tree = ET.parse(str(src))
    root = tree.getroot()

    # 1. mesh 상대경로 → 절대경로 (patched URDF 위치 의존성 제거)
    urdf_dir = src.parent.resolve()
    for mesh_el in root.iter("mesh"):
        filename = mesh_el.get("filename")
        if not filename or filename.startswith(("package://", "file://", "/")):
            continue
        p = Path(filename)
        if p.is_absolute():
            continue
        abs_path = (urdf_dir / filename).resolve()
        mesh_el.set("filename", str(abs_path).replace("\\", "/"))

    # 2. joint origin patch
    n_patched = 0
    for joint_el in root.findall("joint"):
        name = joint_el.get("name", "")
        if name not in joint_id_map:
            continue
        jid = joint_id_map[name]
        origin_el = joint_el.find("origin")
        if origin_el is None:
            continue

        d_trans = offsets.get_trans(jid)
        d_rot = offsets.get_rot(jid)
        if not np.any(d_trans) and not np.any(d_rot):
            continue

        xyz = _parse_xyz(origin_el.get("xyz", "0 0 0"))
        rpy = _parse_xyz(origin_el.get("rpy", "0 0 0"))
        origin_el.set("xyz", _fmt_xyz(xyz + d_trans))
        origin_el.set("rpy", _fmt_xyz(rpy + d_rot))
        n_patched += 1

    if n_patched > 0:
        logger.debug(f"URDF patched: {n_patched} joints")
    return ET.tostring(root, encoding="unicode")
