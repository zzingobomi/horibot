"""빌드 프로세스 격리 e2e — 실 subprocess(spawn) + 실 Open3D build_mesh.

의미 (뒤집으면 회귀): 빌드가 backend 프로세스 안으로 돌아오면 Open3D 가 bridge
릴레이를 굶겨 로봇 스트림이 버벅인다 (2026-07-19 실물 확정 — build_world off
대조 실험). 잠그는 계약: ① 진짜 다른 프로세스에서 돈다 ② progress 가 부모로
릴레이된다 ③ build.py 진단 로그(pair fitness/corr — 정합 사고를 잡는 로그)가
부모 로거로 릴레이된다 ④ 결과는 in-process 와 같은 BuildResult.

sim 마킹 — 워커 spawn + open3d import (수 초). Runtime/Zenoh 없음 (LAN 무해).
"""

from __future__ import annotations

import logging
import os

import numpy as np
import pytest

from modules.scan import isolated_build
from modules.scan.build import BuildScanInput

pytestmark = pytest.mark.sim


def _scan(x_shift: float) -> BuildScanInput:
    """합성 스캔 — 0.3m 평면을 보는 카메라 (x 로 살짝 평행이동한 두 뷰)."""
    w, h = 64, 48
    color = np.full((h, w, 3), 128, dtype=np.uint8)
    depth = np.full((h, w), 300, dtype=np.uint16)  # 0.3m (depth_scale 0.001)
    t = np.eye(4)
    t[0, 3] = x_shift
    return BuildScanInput(
        color_bgr=color, depth_z16=depth, width=w, height=h,
        fx=60.0, fy=60.0, cx=32.0, cy=24.0, depth_scale=0.001,
        t_base_cam_init=t,
    )


async def test_isolated_build_subprocess_relays_progress_and_logs(
    caplog: pytest.LogCaptureFixture,
):
    stages: list[tuple[str, float, str]] = []
    try:
        with caplog.at_level(logging.INFO, logger="modules.scan.build"):
            result = await isolated_build.run_isolated(
                [_scan(0.0), _scan(0.005)],
                {"voxel_size": 0.004, "sdf_trunc": 0.02},
                lambda s, p, m: stages.append((s, p, m)),
            )
        # ① 결과 — in-process 와 동일 계약 (mesh + 통계)
        assert result.n_scans == 2
        assert result.vertex_count > 0 and len(result.mesh_bytes) > 0
        # ② progress 릴레이 — 워커의 stage 콜백이 부모에 도달
        assert stages, "progress 릴레이 0건 — 격리가 관측성을 죽임"
        # ③ 진단 로그 릴레이 — pair 정합 라인 (2026-07-19 사고를 잡은 로그)
        assert any("pair" in r.message for r in caplog.records), (
            "build.py pair 진단 로그가 부모로 릴레이 안 됨"
        )
        # ④ 진짜 별도 프로세스에서 돌았나
        executor, _ = isolated_build._ensure_pool()
        worker_pid = executor.submit(os.getpid).result()
        assert worker_pid != os.getpid()
    finally:
        isolated_build.shutdown()  # 워커/매니저 유령 방지
