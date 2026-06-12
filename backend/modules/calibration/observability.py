"""Hand-Eye 관측성 진단 — IRLS sprint 시작 조건 판정.

trauma 사이클의 root cause 가 두 가설 중 어디인지 데이터로 확정:
- 가설 A: outlier influence unbounded → IRLS+Huber 가 답
- 가설 B: 캘판 보면서 만들 수 있는 카메라 광축 다양성이 원천 부족 → 어떤 solver 든 한계

OMX-F (5DOF, wrist yaw 없음) + 23cm 보드 거리 환경에서 가설 B 가 의심돼
sprint 1주 시작 전 30분 진단으로 정당화 여부 판정.

handeye_poses.npz 의 PnP 결과 (R_target2cam, t_target2cam) 만 사용. BA / FK / hardware
모두 불필요.

Refs:
- Tsai & Lenz 1989 — relative motion 회전축이 3D 공간 span 해야 hand-eye observable
- Andreff et al. — 평면 운동만으로는 translation 일부 unobservable
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from . import thresholds as T


@dataclass
class ObservabilityReport:
    n_poses: int
    # metric 1 — 카메라 광축 펼침 (보드 frame 기준 카메라 forward 8개의 pairwise max angle, deg)
    axis_spread_deg: float
    # metric 2 — 보드 tilt 분포 (deg). 권장 TILT_MIN_DEG ~ TILT_MAX_DEG
    tilt_min_deg: float
    tilt_max_deg: float
    tilt_std_deg: float
    tilt_in_range_count: int  # 권장 범위 안 자세 수
    # metric 3 — relative motion 회전축의 3D span (σ_3/σ_1 of covariance)
    rotation_axis_ratio: float
    # metric 4 — wrist roll (joint5) raw 분포
    wrist_roll_range_raw: int

    def verdict(self) -> str:
        a_signals = (self.axis_spread_deg > 40) + (self.rotation_axis_ratio > 0.3)
        b_signals = (self.axis_spread_deg < 20) + (self.rotation_axis_ratio < 0.1)
        if a_signals >= 2:
            return "A (다양성 충분, IRLS sprint 정당)"
        if b_signals >= 2:
            return "B (구조적 부족, paradigm 분기 검토)"
        return "중간 (signals 섞임, 양쪽 일부)"


def analyze_pose_data(
    R_target2cam: np.ndarray,
    raw_positions: np.ndarray,
    *,
    wrist_roll_axis: int = 4,
) -> ObservabilityReport:
    """in-memory R/t + raw 로 진단 (npz 의존성 X).

    calibration_node 가 capture 후 자동 호출 + observability state publish 위해
    분리. analyze(npz_path) 는 thin wrapper.

    Args:
        R_target2cam: (N, 3, 3) board → cam rotation
        raw_positions: (N, J) Dynamixel raw. raw[:, wrist_roll_axis] 가 wrist roll.
        wrist_roll_axis: wrist roll motor index. OMX-F=4 (joint5). 다른 robot 에서 다름.
    """
    R = np.asarray(R_target2cam)
    raw = np.asarray(raw_positions)
    n = R.shape[0]

    # 1. 카메라 광축 펼침 (in board frame)
    # cam forward = [0,0,1]_cam → board frame: R_cam2board @ [0,0,1] = R_target2cam.T @ [0,0,1]
    # = R_target2cam 의 3번째 row.
    cam_fwd_in_board = R[:, 2, :].copy()  # (N, 3)
    cam_fwd_in_board /= np.linalg.norm(cam_fwd_in_board, axis=1, keepdims=True)
    cos_pairs = np.clip(cam_fwd_in_board @ cam_fwd_in_board.T, -1, 1)
    iu = np.triu_indices(n, k=1)
    angles_deg = np.degrees(np.arccos(cos_pairs[iu]))
    axis_spread = float(angles_deg.max())

    # 2. 보드 tilt = arccos(R[2,2])
    tilt_cos = np.clip(R[:, 2, 2], -1, 1)
    tilt_deg = np.degrees(np.arccos(tilt_cos))
    in_range = int(np.sum((tilt_deg >= T.TILT_MIN_DEG) & (tilt_deg <= T.TILT_MAX_DEG)))

    # 3. relative motion 회전축 spanning (Tsai degeneracy test)
    axes_list: list[np.ndarray] = []
    for i in range(n):
        for j in range(i + 1, n):
            R_ij = R[j] @ R[i].T
            rvec, _ = cv2.Rodrigues(R_ij)
            ang = float(np.linalg.norm(rvec))
            if ang < 1e-6:
                continue
            axes_list.append(rvec.flatten() / ang)
    axes = np.array(axes_list)  # (M, 3)
    # outer product sum — sign 모호함 자동 cancel (v vs -v 같은 outer product)
    M_cov = axes.T @ axes / len(axes)
    eigvals = np.linalg.eigvalsh(M_cov)[::-1]  # descending
    # σ ratio = sqrt(λ3/λ1). PSD numerical noise 가드.
    axis_ratio = float(np.sqrt(max(eigvals[2], 0.0) / eigvals[0]))

    # 4. wrist roll raw 분포
    wrist_roll_range = int(raw[:, wrist_roll_axis].max() - raw[:, wrist_roll_axis].min())

    return ObservabilityReport(
        n_poses=n,
        axis_spread_deg=axis_spread,
        tilt_min_deg=float(tilt_deg.min()),
        tilt_max_deg=float(tilt_deg.max()),
        tilt_std_deg=float(tilt_deg.std()),
        tilt_in_range_count=in_range,
        rotation_axis_ratio=axis_ratio,
        wrist_roll_range_raw=wrist_roll_range,
    )


def analyze(npz_path: Path) -> ObservabilityReport:
    """thin wrapper — npz path 로딩 후 analyze_pose_data 호출."""
    d = np.load(str(npz_path), allow_pickle=True)
    return analyze_pose_data(d["R_target2cam"], d["raw_positions"])


def main() -> None:
    import sys

    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    repo_root = Path(__file__).resolve().parents[3]
    default_path = (
        repo_root / "robot/instances/omx_f_0/calibration/handeye_poses.npz"
    )
    rep = analyze(default_path)

    print("=== Hand-Eye Observability Diagnosis ===")
    print(f"source              : {default_path.name}")
    print(f"n_poses             : {rep.n_poses}")
    print()
    print("--- metrics ---")
    print(
        f"axis_spread_deg     : {rep.axis_spread_deg:6.2f}°  "
        f"(A>40 / B<20)"
    )
    print(
        f"tilt range          : {rep.tilt_min_deg:5.1f}° ~ {rep.tilt_max_deg:5.1f}°  "
        f"std={rep.tilt_std_deg:.1f}°  "
        f"in [{T.TILT_MIN_DEG}, {T.TILT_MAX_DEG}]: "
        f"{rep.tilt_in_range_count}/{rep.n_poses}"
    )
    print(
        f"rotation_axis_ratio : {rep.rotation_axis_ratio:6.3f}  "
        f"(A>0.3 / B<0.1)"
    )
    print(
        f"wrist_roll_range    : {rep.wrist_roll_range_raw:6d} raw  "
        f"({np.degrees(rep.wrist_roll_range_raw / 4095 * 2 * np.pi):.0f}°)"
    )
    print()
    print(f"verdict: {rep.verdict()}")


if __name__ == "__main__":
    main()
