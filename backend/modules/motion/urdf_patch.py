"""link_offset → patched URDF 생성 (Motion D4).

calibration 의 LinkOffset (arm joint 별 origin xyz 가산 + rotvec 회전) 을 URDF
파일에 구워 PybulletKinematics 가 그대로 로드 (PyBullet 은 load 후 joint origin
변경 불가 — fk_chain.py 참조).

**의미 SSOT = FkChain._joint_origin_with_offset** (offline BA 가 이 의미로 추정):
    T_o[:3, 3] += trans_m               # parent frame 에서 xyz 가산
    T_o[:3, :3] = T_o[:3, :3] @ R(rotvec)   # 기존 회전 뒤에 우측 곱

동일 의미 검증 test = tests/modules/test_urdf_patch.py (FkChain(patched) ==
FkChain(orig, link_offsets)).

산출 파일은 원본과 같은 디렉토리 (`<type>.<robot_id>.calibrated.urdf`) — URDF 의
상대 mesh 경로 보존. gitignore (런타임 산출물).
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)


def _parse_floats(s: str | None, n: int) -> np.ndarray:
    if not s:
        return np.zeros(n)
    vals = [float(x) for x in s.split()]
    if len(vals) != n:
        raise ValueError(f"origin 속성 원소 수 {len(vals)} != {n}: {s!r}")
    return np.asarray(vals, dtype=np.float64)


def _fmt(vals: np.ndarray) -> str:
    return " ".join(f"{v:.12g}" for v in vals)


def patch_urdf_link_offsets(
    urdf_path: Path,
    robot_id: str,
    joint_offsets: dict[str, tuple[list[float], list[float]]],
) -> Path:
    """URDF 의 지정 joint origin 에 link_offset 적용 후 sibling 파일로 저장.

    Args:
        urdf_path: 원본 URDF.
        robot_id: 산출 파일명 구분 (같은 type N대 충돌 방지).
        joint_offsets: joint name → (trans_m[3], rot_rad rotvec[3]).

    Returns:
        patched URDF 경로 (원본과 같은 디렉토리).
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    remaining = dict(joint_offsets)
    for joint in root.iter("joint"):
        name = joint.get("name")
        if name not in remaining:
            continue
        trans, rotvec = remaining.pop(name)
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        xyz = _parse_floats(origin.get("xyz"), 3)
        rpy = _parse_floats(origin.get("rpy"), 3)

        # FkChain 의미: translation 은 parent frame 가산
        xyz = xyz + np.asarray(trans, dtype=np.float64)
        # rotation 은 기존 R 뒤 우측 곱. URDF rpy = fixed-axis(extrinsic) XYZ
        # = scipy 소문자 "xyz".
        rv = np.asarray(rotvec, dtype=np.float64)
        if np.any(rv):
            R_orig = Rotation.from_euler("xyz", rpy)
            R_new = R_orig * Rotation.from_rotvec(rv)
            rpy = R_new.as_euler("xyz")

        origin.set("xyz", _fmt(xyz))
        origin.set("rpy", _fmt(rpy))

    if remaining:
        raise ValueError(
            f"URDF 에 없는 joint 에 link_offset: {sorted(remaining)} ({urdf_path})"
        )

    out = urdf_path.with_name(f"{urdf_path.stem}.{robot_id}.calibrated.urdf")
    tree.write(out, encoding="unicode")
    logger.info("patched URDF 생성: %s (joints=%d)", out, len(joint_offsets))
    return out
