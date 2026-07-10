"""Detector base-투영 수학 단위테스트 (결정적, 모델/하드웨어 무관).

의미 있는 검증: 알려진 카메라 pose + intrinsic 에서 pixel → base 좌표가 손계산과
일치하는지. Grounding DINO / 실 하드웨어 없이 회사에서 투영 정확성 검증.
"""

from __future__ import annotations

import numpy as np

from modules.detector.projection import (
    base_points_from_mask,
    floor_z_and_height,
    object_top_center_base,
    project_base_to_pixel,
    unproject_to_base,
    z_cam_from_depth_bbox,
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


def test_z_cam_from_depth_bbox_top_percentile():
    # 100x100 depth, bbox 안에 물체(가까움, raw 1000) + 배경(멀리, raw 2000).
    depth = np.full((100, 100), 2000, dtype=np.uint16)
    depth[40:60, 40:60] = 1000  # 물체 (카메라에 가까움)
    z = z_cam_from_depth_bbox(depth, (40, 40, 60, 60), depth_scale=0.001)
    # ROI 전부 1000 → percentile25 = 1000 → 1.0m
    assert z is not None and abs(z - 1.0) < 1e-6, z


def test_object_top_center_uses_top_face_not_bbox_center():
    """윗면 픽셀 3D centroid = 윗면 중심 (bbox 중심 픽셀 편향 fix, 결정적).

    비스듬한 시점: bbox 안에서 윗면(가까운 depth)이 위쪽에 치우치고 아래는 옆면(더 멂).
    옛 방식(bbox 중심 픽셀 + 윗면 depth)은 파지 x/y 를 아래(카메라 쪽)로 밀지만,
    윗면 픽셀들의 실제 centroid 는 진짜 윗면 중심을 복원. 이 assert 를 옛 방식으로
    되돌리면(bbox 중심) 즉시 깨짐.
    """
    fx = fy = 500.0
    cx = cy = 100.0
    r_be = np.diag([1.0, -1.0, -1.0])  # 카메라 수직 내려봄
    t_be = np.array([0.0, 0.0, 0.5])
    depth = np.zeros((200, 200), dtype=np.uint16)
    depth[80:100, 80:120] = 200  # 윗면 (가까움, base_z 0.3) — bbox 위쪽 절반
    depth[100:120, 80:120] = 230  # 옆면 (더 멂, base_z 0.27) — bbox 아래 절반
    bbox = (80.0, 80.0, 120.0, 120.0)  # 중심 픽셀 (100, 100)

    out = object_top_center_base(
        depth, bbox, 0.001, fx, fy, cx, cy, r_be, t_be, np.eye(3), np.zeros(3)
    )
    assert out is not None
    # 윗면 픽셀 centroid = (u=99.5, v=89.5) @ z_cam 0.2 → 이걸 복원해야.
    expect = unproject_to_base(
        99.5, 89.5, 0.2, fx, fy, cx, cy, r_be, t_be, np.eye(3), np.zeros(3)
    )
    assert np.allclose(out, expect, atol=1e-3), (out, expect)
    # 옛 방식(bbox 중심 픽셀 v=100)과 유의미하게 다름 = 편향 제거 증명.
    old_biased = unproject_to_base(
        100.0, 100.0, 0.2, fx, fy, cx, cy, r_be, t_be, np.eye(3), np.zeros(3)
    )
    assert abs(out[1] - old_biased[1]) > 3e-3, (out[1], old_biased[1])


def test_object_top_center_no_valid_returns_none():
    depth = np.zeros((50, 50), dtype=np.uint16)
    assert object_top_center_base(
        depth, (10.0, 10.0, 30.0, 30.0), 0.001, 500.0, 500.0, 25.0, 25.0,
        np.eye(3), np.zeros(3), np.eye(3), np.zeros(3),
    ) is None


def test_z_cam_from_depth_bbox_no_valid_returns_none():
    depth = np.zeros((50, 50), dtype=np.uint16)  # valid depth 없음
    assert z_cam_from_depth_bbox(depth, (10, 10, 30, 30), 0.001) is None
    # 무효 bbox
    assert z_cam_from_depth_bbox(np.ones((50, 50), np.uint16), (30, 30, 10, 10), 0.001) is None


def test_floor_z_and_height_ring():
    # 카메라가 base 를 내려다봄 (cam +z → base -z): R_be=diag(1,-1,-1), t_be z=0.5.
    # 물체 윗면 depth 200 (z_cam 0.2) → base_z=-0.2+0.5=0.3. 주변 책상 depth 250
    # (z_cam 0.25) → base_z=-0.25+0.5=0.25. height = 0.3 - 0.25 = 0.05.
    depth = np.full((200, 200), 250, dtype=np.uint16)  # 전체 = 책상 (멀리)
    depth[90:110, 90:110] = 200  # 중앙 = 물체 (카메라에 가까움)
    fx = fy = 500.0
    cx, cy = 100.0, 100.0
    r_be = np.diag([1.0, -1.0, -1.0])
    t_be = np.array([0.0, 0.0, 0.5])
    floor_z, height = floor_z_and_height(
        depth, (90.0, 90.0, 110.0, 110.0), 0.001, fx, fy, cx, cy,
        r_be, t_be, np.eye(3), np.zeros(3), obj_top_base_z=0.3,
    )
    assert abs(floor_z - 0.25) < 1e-6, floor_z
    assert abs(height - 0.05) < 1e-6, height


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


def test_floor_z_and_height_no_ring_returns_top():
    # ring 에 valid depth 없음 (물체만 있고 주변 0) → floor_z=obj_top, height=0.
    depth = np.zeros((200, 200), dtype=np.uint16)
    depth[90:110, 90:110] = 200
    floor_z, height = floor_z_and_height(
        depth, (90.0, 90.0, 110.0, 110.0), 0.001, 500.0, 500.0, 100.0, 100.0,
        np.eye(3), np.zeros(3), np.eye(3), np.zeros(3), obj_top_base_z=0.7,
    )
    assert floor_z == 0.7 and height == 0.0, (floor_z, height)
