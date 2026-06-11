"""원본 URDF에 link_offsets를 patch 적용해 PyBullet이 로드할 URDF 생성.

설계 이유:
    PyBullet의 loadURDF는 파일 경로만 받음 (메모리 string API 없음). 그렇다고
    원본 URDF를 매번 modify-in-place하면 git status가 노이지하고 분산 모드에서
    "URDF는 source-of-truth + link_offsets.npz만 push" 흐름이 깨짐.

해결:
    원본 URDF는 *그대로* 두고, 머신 부팅 시 link_offsets.npz를 patch 적용한
    URDF를 .patched/ 폴더(gitignored)에 생성. PyBullet은 그 patched 파일을 로드.
    다른 머신은 git pull + 재시작 → 자체 patched URDF 갱신.

URDF rpy 가산 — small-angle 가정:
    BA가 추정하는 link_rot은 axis-angle rotation vector (rad). URDF의
    <joint><origin rpy="..."/>는 ZYX 오일러. 일반적으로 두 표현이 다르나, 작은
    각(<5°)에서는 rpy ≈ rotvec 근사 정확. 현재 v3 final 결과 link_rot 최대
    0.85° 수준이라 안전. 큰 각이 필요해지면 정확한 matrix→rpy 변환 도입.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from modules.calibration.link_offsets import LinkOffsets

logger = logging.getLogger(__name__)


def _default_joint_id_map() -> dict[str, int]:
    """URDF joint name → link_offsets dict key. joint1~joint5 → 1~5 (모터 ID와 동일).

    tcp_joint 는 *URDF patch 안 함* — 그 위치는 캘리브레이션 reference frame
    (hand_eye / IK 의 기준점) 으로 고정. URDF EE (tcp) 와 실제 그리퍼 끝점 사이
    갭은 LinkCoordinates ID=6 행을 *tool_offset* 으로 재해석해 motion_node 의
    service handler 에서 명령 좌표 변환에 사용 (cancel out 회피).

    이력: 한때 tcp_joint:6 으로 URDF patch 시도했으나 detect 와 IK 양쪽 모두
    같은 patched URDF 위에 도는 self-consistency 로 cancel out 됨 → 효과 0
    (2026-05-28 22:18 실측 확인). 그 fix 가 이 변경.
    """
    return {f"joint{i}": i for i in range(1, 6)}


def _parse_xyz(s: str) -> np.ndarray:
    parts = s.strip().split()
    return np.array([float(p) for p in parts[:3]], dtype=np.float64)


def _fmt_xyz(v: np.ndarray) -> str:
    return f"{v[0]:.9g} {v[1]:.9g} {v[2]:.9g}"


def patch_urdf_text(
    source_urdf_path: str | Path,
    offsets: LinkOffsets,
    joint_id_map: dict[str, int] | None = None,
) -> str:
    """원본 URDF 파일을 읽어 link_offsets patch 적용한 URDF 텍스트 반환.

    부수 변경: <mesh filename="...">의 상대경로를 절대경로로 rewrite.
    patched URDF가 원본과 다른 디렉토리(.patched/)에 저장돼도 mesh 찾도록.

    Args:
        source_urdf_path: 원본 .urdf 경로.
        offsets: LinkOffsets — joint id별 (link_trans m, link_rot rad rotvec).
        joint_id_map: URDF joint name → motor id 매핑.
            None이면 OMX_F 기본 (joint1→1, ..., joint5→5).

    Returns:
        patched URDF text (utf-8 encoding 가능).
    """
    if joint_id_map is None:
        joint_id_map = _default_joint_id_map()

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


def patched_urdf_path(source_urdf_path: str | Path) -> Path:
    """patched URDF가 저장될 경로.

    `<urdf_dir>/.patched/<original_name>.urdf` — .gitignore된 디렉토리.
    """
    src = Path(source_urdf_path)
    return src.parent / ".patched" / src.name


def write_patched_urdf(
    source_urdf_path: str | Path,
    offsets: LinkOffsets,
    joint_id_map: dict[str, int] | None = None,
) -> Path:
    """patched URDF를 표준 경로(.patched/)에 저장하고 그 경로 반환.

    offsets가 비어 있으면 원본 그대로 복사 (PyBullet이 한 경로만 가리키도록).
    """
    src = Path(source_urdf_path)
    out = patched_urdf_path(src)
    out.parent.mkdir(parents=True, exist_ok=True)
    text = patch_urdf_text(src, offsets, joint_id_map)
    out.write_text(text, encoding="utf-8")
    return out
