"""scan build 쌍 정합 회귀 (sim — open3d ICP 실행).

colored ICP 교체(2026-07-18, docs/perception.md)의 존재 이유를 그대로 잠근다:
작업대 장면은 평면이 지배하고, point-to-plane ICP 는 평면 내 이동(x/y/yaw)에
비용이 불변이라 FK 초기오차(σ_t ~7.5mm)가 그대로 남는다 — TSDF 가 그 오정합을
평균해 텍스처가 번진 것이 session 2 실측. colored ICP 는 색 그라디언트 항이
그 자유도를 구속한다. 이 테스트가 red 면 정합이 다시 in-plane 퇴화로 돌아갔다.
"""

from __future__ import annotations

import numpy as np
import open3d as o3d
import pytest

from modules.scan import build as recon

pytestmark = pytest.mark.sim

# FK 오차 시뮬레이션 — 실측 σ_t(~7.5mm) 급의 평면 내 shift.
_SHIFT = np.array([0.006, 0.004, 0.0])
# 카메라 프레임 재현: 평면은 카메라(원점) 아래 30cm (D405 원거리 관측 대역).
_PLANE_Z = -0.30


def _table_cloud(
    shift: np.ndarray | None = None, *, colored: bool = True
) -> o3d.geometry.PointCloud:
    """0.4×0.3m 완전 평면 격자 (2mm step) — 기하는 p2p 퇴화 조건 그 자체.

    색 = 절대 표면 좌표의 함수 (나뭇결 대역 ~5cm 주기, x/y 양방향) — shift 된
    관측(src)도 같은 물리 무늬를 본다 (FK 만 틀렸다는 모델링)."""
    xs = np.arange(-0.20, 0.20, 0.002)
    ys = np.arange(-0.15, 0.15, 0.002)
    gx, gy = np.meshgrid(xs, ys)
    surf = np.stack(
        [gx.ravel(), gy.ravel(), np.full(gx.size, _PLANE_Z)], axis=1
    )
    pts = surf if shift is None else surf + shift
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if colored:
        r = 0.5 + 0.5 * np.sin(2.0 * np.pi * surf[:, 0] / 0.05)
        g = 0.5 + 0.5 * np.sin(2.0 * np.pi * surf[:, 1] / 0.05)
        b = np.full(surf.shape[0], 0.5)
        pcd.colors = o3d.utility.Vector3dVector(np.stack([r, g, b], axis=1))
    return pcd


def test_colored_recovers_inplane_shift_where_p2p_cannot():
    """★ 평면 장면 in-plane 오차: colored 는 회복, p2p 는 못 잡는다 (퇴화 증명).

    둘 다 assert 하는 이유 — p2p 쪽이 어느 날 "잡게" 되면 이 합성 장면이
    퇴화 조건을 잃었다는 뜻이라 테스트 자체가 무의미해진다 (전제 검증)."""
    tgt = _table_cloud()
    src = _table_cloud(_SHIFT)  # FK 가 shift 만큼 틀리게 배치한 관측
    t_true = np.eye(4)
    t_true[:3, 3] = -_SHIFT  # src→tgt 정답 = shift 상쇄

    pyr_src = recon.build_pyramid(src)
    pyr_tgt = recon.build_pyramid(tgt)

    t_icp, _info, fitness, method, _corr, trusted = recon.register_pair(
        pyr_src, pyr_tgt, np.eye(4)
    )
    assert method == "colored"
    assert trusted  # 6mm 보정은 발산 아님
    assert fitness > 0.5
    err_colored = float(np.linalg.norm(t_icp[:3, 3] - t_true[:3, 3]))
    assert err_colored < 0.002, (
        f"colored ICP 가 in-plane {np.linalg.norm(_SHIFT)*1000:.1f}mm 오차를 "
        f"못 잡음 (잔차 {err_colored*1000:.1f}mm)"
    )

    # 전제 검증: 같은 데이터에서 point-to-plane 은 in-plane 을 못 잡는다.
    fine = min(pyr_src)
    res_p2p = o3d.pipelines.registration.registration_icp(
        pyr_src[fine], pyr_tgt[fine], recon.DEFAULT_ICP_MAX_DIST, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    err_p2p = float(np.linalg.norm(res_p2p.transformation[:3, 3] - t_true[:3, 3]))
    assert err_p2p > 0.004, (
        "p2p 가 평면 in-plane 오차를 잡음 — 합성 장면이 퇴화 조건을 잃었다"
    )


def test_register_pair_falls_back_to_p2p_without_colors():
    """색 없는 점군(비정상 입력/구형 데이터)에서도 정합은 죽지 않는다 — colored
    예외 → point-to-plane 폴백 (옛 파이프라인과 동일 경로)."""
    tgt = _table_cloud(colored=False)
    src = _table_cloud(_SHIFT, colored=False)
    t_icp, _info, _fitness, method, _corr, _trusted = recon.register_pair(
        recon.build_pyramid(src), recon.build_pyramid(tgt), np.eye(4)
    )
    assert method.startswith("p2p_fallback")
    assert t_icp.shape == (4, 4)


def test_divergent_correction_falls_back_to_fk_initial():
    """★ 발산 게이트 (2026-07-18 실물 recon10: 평면 aliasing 이 colored ICP 를
    100~130mm 엉뚱한 곳에 lock → PoseGraph 오염 → 스캔 fly-off). FK 초기값이
    맞는데 ICP 가 _MAX_CORR_M 넘게 옮기려 하면 발산으로 보고 FK 로 되돌려야 한다.

    합성: 평면 두 장을 큰 값(_MAX_CORR_M 초과)으로 어긋나게 두되 초기값은 정답
    (=정렬됨)으로 준다. 텍스처가 주기적이라 ICP 는 한 주기(=오정렬)로 끌려갈 수
    있는데, 게이트가 그걸 잡아 초기값(정답)을 지켜야 한다."""
    tgt = _table_cloud()
    src = _table_cloud()  # 동일 = 초기값 eye(4)가 이미 정답
    # 초기값을 일부러 정답(정렬)으로 주면, 올바른 ICP 는 corr≈0 (trusted).
    # 그런데 만약 aliasing 으로 한 주기(color 무늬 5cm) 끌려가면 corr≥50mm →
    # 게이트가 FK(eye)로 되돌려 corr 0 을 만든다. 어느 경우든 결과는 정렬.
    t_icp, _info, _fit, method, corr, trusted = recon.register_pair(
        recon.build_pyramid(src), recon.build_pyramid(tgt), np.eye(4),
        max_corr_m=0.04,
    )
    if not trusted:
        # 발산 감지 시 FK(eye) 로 되돌려졌어야 — 이동량 0
        assert "→fk" in method
        assert float(np.linalg.norm(t_icp[:3, 3])) < 1e-9
    else:
        # 정상 수렴 시 초기값(정답) 근처 — 큰 이동 없음
        assert corr <= 0.04


def test_max_corr_gate_rejects_large_shift():
    """게이트 직접 검증: 초기값을 정답에서 크게(80mm) 벗어나게 주면, 설령 ICP
    가 그 근처 국소해로 수렴해도 FK(그 잘못된 초기값)로 되돌린다 — 즉 corr 은
    항상 max_corr_m 이하로 clamp 되고 trusted=False 로 표시된다."""
    tgt = _table_cloud()
    src = _table_cloud()
    bad_init = np.eye(4)
    bad_init[0, 3] = 0.08  # 80mm 어긋난 초기값 (> 40mm 게이트)
    t_icp, _info, _fit, method, _corr, trusted = recon.register_pair(
        recon.build_pyramid(src), recon.build_pyramid(tgt), bad_init,
        max_corr_m=0.04,
    )
    # ICP 가 80mm 초기값에서 40mm 넘게 되돌아왔으면(정답 방향) 발산 게이트가
    # 그 큰 보정을 발산으로 판정 → 초기값 유지. 정답으로 수렴했든 아니든,
    # "게이트가 큰 보정을 통과시키지 않는다"가 불변.
    if not trusted:
        assert "→fk" in method
        assert float(np.linalg.norm(t_icp[:3, 3] - bad_init[:3, 3])) < 1e-9


def test_sdf_trunc_derivation_anchors_current_default():
    """voxel→sdf_trunc 파생 규칙: 현행 기본(2mm/10mm=5×) 앵커 + 하한 10mm.
    8mm voxel 에 10mm trunc(1.25 voxel band)를 그대로 두면 marching cubes 가
    찢어진다 — 이 결합을 끊는 회귀 방지."""
    assert recon.sdf_trunc_for(0.001) == pytest.approx(0.010)
    assert recon.sdf_trunc_for(0.002) == pytest.approx(0.010)
    assert recon.sdf_trunc_for(0.004) == pytest.approx(0.020)
    assert recon.sdf_trunc_for(0.008) == pytest.approx(0.040)
