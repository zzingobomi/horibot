"""Detector base-투영 수학 단위테스트 (결정적, 모델/하드웨어 무관).

의미 있는 검증: 알려진 카메라 pose + intrinsic 에서 pixel → base 좌표가 손계산과
일치하는지. Grounding DINO / 실 하드웨어 없이 회사에서 투영 정확성 검증.
"""

from __future__ import annotations

import numpy as np

from modules.detector.projection import unproject_to_base, z_cam_from_depth_bbox


def test_center_pixel_identity_pose():
    # 카메라=ee=base 정렬(R=I), base 원점 위 0.5m 에 카메라. 중심 픽셀 → 광축 위.
    fx = fy = 600.0
    cx, cy = 320.0, 240.0
    z_cam = 0.3
    r_be = np.eye(3)
    t_be = np.array([0.0, 0.0, 0.5])
    r_ce = np.eye(3)
    t_ce = np.zeros(3)
    out = unproject_to_base(cx, cy, z_cam, fx, fy, cx, cy, r_be, t_be, r_ce, t_ce)
    # 중심 픽셀 → X=Y=0, Z=z_cam. ee=cam identity → base = [0,0,0.5+0.3]
    assert np.allclose(out, [0.0, 0.0, 0.8], atol=1e-9), out


def test_offset_pixel_projects_linearly():
    fx = fy = 500.0
    cx, cy = 320.0, 240.0
    z_cam = 0.4
    # +32px x, +25px y → X = 32/500*0.4, Y = 25/500*0.4
    u, v = cx + 32.0, cy + 25.0
    r_be = np.eye(3)
    t_be = np.zeros(3)
    r_ce = np.eye(3)
    t_ce = np.zeros(3)
    out = unproject_to_base(u, v, z_cam, fx, fy, cx, cy, r_be, t_be, r_ce, t_ce)
    assert np.allclose(out, [32 / 500 * 0.4, 25 / 500 * 0.4, 0.4], atol=1e-9), out


def test_hand_eye_translation_offset_applied():
    # cam→ee 순수 평행이동 (카메라가 ee 앞 +z 0.05m). base=ee identity.
    fx = fy = 600.0
    cx, cy = 320.0, 240.0
    r_be = np.eye(3)
    t_be = np.zeros(3)
    r_ce = np.eye(3)
    t_ce = np.array([0.0, 0.0, 0.05])
    out = unproject_to_base(cx, cy, 0.2, fx, fy, cx, cy, r_be, t_be, r_ce, t_ce)
    # obj_cam=[0,0,0.2] → obj_ee=[0,0,0.25] → obj_base 동일
    assert np.allclose(out, [0.0, 0.0, 0.25], atol=1e-9), out


def test_base_rotation_applied():
    # ee→base 가 z축 90° 회전. cam=ee identity. obj_cam=[0,0,0.2] 는 축 위라 회전 불변.
    fx = fy = 600.0
    cx, cy = 320.0, 240.0
    theta = np.pi / 2
    r_be = np.array(
        [
            [np.cos(theta), -np.sin(theta), 0.0],
            [np.sin(theta), np.cos(theta), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    t_be = np.zeros(3)
    # 광축 위 점은 z회전 불변 → [0,0,0.2]
    out = unproject_to_base(
        cx, cy, 0.2, fx, fy, cx, cy, r_be, t_be, np.eye(3), np.zeros(3)
    )
    assert np.allclose(out, [0.0, 0.0, 0.2], atol=1e-9), out
    # off-axis 점은 회전됨: obj_ee=[dx,0,z] → base=[0,dx,z]
    u = cx + 60.0
    out2 = unproject_to_base(
        u, cy, 0.2, fx, fy, cx, cy, r_be, t_be, np.eye(3), np.zeros(3)
    )
    dx = 60.0 / 600.0 * 0.2
    assert np.allclose(out2, [0.0, dx, 0.2], atol=1e-9), out2


def test_z_cam_from_depth_bbox_top_percentile():
    # 100x100 depth, bbox 안에 물체(가까움, raw 1000) + 배경(멀리, raw 2000).
    depth = np.full((100, 100), 2000, dtype=np.uint16)
    depth[40:60, 40:60] = 1000  # 물체 (카메라에 가까움)
    z = z_cam_from_depth_bbox(depth, (40, 40, 60, 60), depth_scale=0.001)
    # ROI 전부 1000 → percentile25 = 1000 → 1.0m
    assert z is not None and abs(z - 1.0) < 1e-6, z


def test_z_cam_from_depth_bbox_no_valid_returns_none():
    depth = np.zeros((50, 50), dtype=np.uint16)  # valid depth 없음
    assert z_cam_from_depth_bbox(depth, (10, 10, 30, 30), 0.001) is None
    # 무효 bbox
    assert z_cam_from_depth_bbox(np.ones((50, 50), np.uint16), (30, 30, 10, 10), 0.001) is None
