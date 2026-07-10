"""RGBD → Open3D PointCloud + 라이브 wire 인코딩 — 옛 scene3d_node 이월.

CRITICAL (옛 gotcha 이월):
- depth_scale 은 create_from_color_and_depth 에 **역수**로 (1/depth_scale).
- Open3D 는 RGB — BGR → RGB 변환 필수.
"""

from __future__ import annotations

import cv2
import numpy as np
import open3d as o3d

from .contract import Scene3dIntrinsic

DEPTH_TRUNC = 1.0  # m — 라이브 stream truncation (D405 근거리)


def _pinhole(intr: Scene3dIntrinsic) -> o3d.camera.PinholeCameraIntrinsic:
    return o3d.camera.PinholeCameraIntrinsic(
        width=intr.width,
        height=intr.height,
        fx=intr.fx,
        fy=intr.fy,
        cx=intr.cx,
        cy=intr.cy,
    )


def build_pcd(
    color_bgr: np.ndarray,
    depth_z16: np.ndarray,
    intr: Scene3dIntrinsic,
    *,
    depth_trunc: float = DEPTH_TRUNC,
) -> o3d.geometry.PointCloud:
    """aligned color(BGR) + depth(uint16) + intrinsic → camera-frame PointCloud."""
    # depth 해상도에 color 맞춤 (D405 aligned 면 동일, 안전상 resize)
    if color_bgr.shape[:2] != depth_z16.shape[:2]:
        color_bgr = cv2.resize(
            color_bgr,
            (depth_z16.shape[1], depth_z16.shape[0]),
            interpolation=cv2.INTER_AREA,
        )
    rgb = np.ascontiguousarray(color_bgr[:, :, ::-1])  # BGR → RGB
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb),
        o3d.geometry.Image(np.ascontiguousarray(depth_z16)),
        depth_scale=1.0 / intr.depth_scale,  # CRITICAL: 역수
        depth_trunc=depth_trunc,
        convert_rgb_to_intensity=False,
    )
    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, _pinhole(intr))


def encode_cloud(pcd: o3d.geometry.PointCloud) -> tuple[int, bytes, bytes]:
    """PointCloud → (point_count, xyz_bytes[N*3 f32], rgb_bytes[N*3 u8])."""
    xyz = np.asarray(pcd.points, dtype=np.float32)
    rgb_f = np.asarray(pcd.colors, dtype=np.float32)
    n = int(xyz.shape[0])
    if n == 0:
        return 0, b"", b""
    rgb_u8 = (np.clip(rgb_f, 0.0, 1.0) * 255.0).astype(np.uint8)
    return n, xyz.tobytes(), rgb_u8.tobytes()


def scale_intrinsic(
    intr: Scene3dIntrinsic, target_w: int, target_h: int, depth_scale: float
) -> Scene3dIntrinsic:
    """intrinsic 을 target 해상도로 scale (active intrinsic 해상도 ≠ depth 해상도 대비)."""
    if intr.width == target_w and intr.height == target_h:
        return Scene3dIntrinsic(
            width=target_w,
            height=target_h,
            fx=intr.fx,
            fy=intr.fy,
            cx=intr.cx,
            cy=intr.cy,
            depth_scale=depth_scale,
        )
    sx = target_w / intr.width
    sy = target_h / intr.height
    return Scene3dIntrinsic(
        width=target_w,
        height=target_h,
        fx=intr.fx * sx,
        fy=intr.fy * sy,
        cx=intr.cx * sx,
        cy=intr.cy * sy,
        depth_scale=depth_scale,
    )
