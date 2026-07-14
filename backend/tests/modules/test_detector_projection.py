"""Detector base-투영 수학 단위테스트 (결정적, 모델/하드웨어 무관).

의미 있는 검증: 알려진 카메라 pose + intrinsic 에서 pixel → base 좌표가 손계산과
일치하는지. Grounding DINO / 실 하드웨어 없이 회사에서 투영 정확성 검증.
"""

from __future__ import annotations

import numpy as np

from modules.detector.projection import (
    base_points_from_mask,
    project_base_to_pixel,
    unproject_to_base,
)


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


def test_base_points_from_mask_matches_unproject():
    # mask 픽셀 각자를 base 로 unproject → unproject_to_base 와 픽셀별 일치 + depth 0 제외.
    fx = fy = 500.0
    cx = cy = 100.0
    r_be = np.eye(3)
    t_be = np.array([0.0, 0.0, 0.5])
    depth = np.zeros((50, 50), dtype=np.uint16)
    mask = np.zeros((50, 50), dtype=bool)
    depth[10, 20] = 300  # (u=20, v=10) z=0.3
    mask[10, 20] = True
    depth[30, 40] = 400  # (u=40, v=30) z=0.4
    mask[30, 40] = True
    mask[5, 5] = True  # mask 이지만 depth 0 → 제외되어야
    pts = base_points_from_mask(
        depth, mask, 0.001, fx, fy, cx, cy, r_be, t_be, np.eye(3), np.zeros(3)
    )
    assert pts is not None and pts.shape == (2, 3), pts
    # np.nonzero row-major 순서 → (10,20) 먼저, (30,40) 다음
    e1 = unproject_to_base(20, 10, 0.3, fx, fy, cx, cy, r_be, t_be, np.eye(3), np.zeros(3))
    e2 = unproject_to_base(40, 30, 0.4, fx, fy, cx, cy, r_be, t_be, np.eye(3), np.zeros(3))
    assert np.allclose(pts[0], e1, atol=1e-9), (pts[0], e1)
    assert np.allclose(pts[1], e2, atol=1e-9), (pts[1], e2)


def test_project_base_to_pixel_inverts_unproject():
    # unproject 로 base 로 보낸 픽셀을 project 로 되돌리면 원 픽셀 (round-trip).
    fx = fy = 550.0
    cx, cy = 320.0, 240.0
    r_be = np.diag([1.0, -1.0, -1.0])  # 비자명 pose
    t_be = np.array([0.1, 0.0, 0.5])
    r_ce = np.eye(3)
    t_ce = np.array([0.0, 0.0, 0.05])
    pixels = [(320.0, 240.0), (400.0, 180.0), (250.0, 300.0)]
    bases = np.array([
        unproject_to_base(u, v, 0.35, fx, fy, cx, cy, r_be, t_be, r_ce, t_ce)
        for u, v in pixels
    ])
    back = project_base_to_pixel(bases, fx, fy, cx, cy, r_be, t_be, r_ce, t_ce)
    assert np.allclose(back, np.array(pixels), atol=1e-6), (back, pixels)


def test_base_points_from_mask_no_valid_returns_none():
    depth = np.zeros((10, 10), dtype=np.uint16)  # depth 전무
    mask = np.ones((10, 10), dtype=bool)
    assert base_points_from_mask(
        depth, mask, 0.001, 500.0, 500.0, 5.0, 5.0,
        np.eye(3), np.zeros(3), np.eye(3), np.zeros(3),
    ) is None
