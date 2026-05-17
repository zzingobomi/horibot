export type HandEyePreview = {
  timestamp: number;
  detected: boolean;
  image_size?: [number, number];
  corners?: [number, number][];
  bbox?: [number, number, number, number];
  coverage_ratio?: number; // 체커보드가 화면에서 차지하는 비율 (너무 작거나 크면 PnP 부정확)
  reason?: string;
};

/**
 * 캡처한 자세 1개의 메타데이터.
 */
export type PoseMeta = {
  index: number;
  timestamp: number;
  joint_angles_rad: number[];
};

/**
 * "알고리즘끼리 결과가 얼마나 일치하나" 표의 한 행.
 *
 * 같은 자세 데이터를 TSAI/PARK/DANIILIDIS로 각각 풀어,
 * 기준(보통 TSAI) 대비 회전/평행이동 차이를 비교한 것.
 *
 * 판정 기준:
 * - 셋 다 Δrot < 1° → 데이터 self-consistent (캘 OK)
 * - 1~3°            → 자세 다양성 부족
 * - > 3°            → 자세 품질 문제 (흔들림 / PnP 실패성 자세)
 */
export type MethodCompareEntry = {
  method: string;
  drot_deg: number; // 기준 대비 회전 차이 (도)
  dt_mm: number; // 기준 대비 평행이동 차이 (mm)
  ref?: boolean; // true면 이 항목이 기준점 (drot/dt는 0)
};

/**
 * 자세별 잔차 한 줄. 값이 크면 outlier 후보 → 개별 삭제 후 재계산.
 *
 * - ComputeData에선  AX=XB 잔차
 * - ValidateData에선 T_base←board의 평균 대비 흩어짐
 */
export type PerPoseResidual = {
  index: number;
  drot_deg: number;
  dt_mm: number;
};

/**
 * `CALIB_HANDEYE_COMPUTE` 응답.
 *
 * "계산만" 수행한 결과 — 파일 저장(COMMIT) 전 미리보기용.
 * 사용자는 이 결과를 보고 outlier 자세를 빼거나 다시 캡처 후 재계산하고,
 * 만족스러우면 별도로 COMMIT을 눌러 hand_eye.npz에 기록.
 */
export type ComputeData = {
  R_cam2gripper: number[][];
  t_cam2gripper: number[];
  method: string;
  pose_count: number;
  method_compare: MethodCompareEntry[];
  per_pose_residual: PerPoseResidual[];
  sigma_rot_deg: number; // 회전 잔차의 표준편차 (도)
  sigma_t_mm: number; // 평행이동 잔차의 표준편차 (mm)
};

/**
 * `CALIB_HANDEYE_VALIDATE` 응답.
 *
 * 같은 캡처 포즈들을 가지고 T_base←board가 자세별로 얼마나 흩어지는지 측정.
 * 좋은 캘이면 모두 같은 값(체커보드는 안 움직였으니까).
 * 흩어짐 = (hand-eye 오차) + (FK 오차). σ가 충분히 작으면 캘 OK, 크면 캘 실패 또는 자세 품질 문제.
 *
 * 목표치: σ_rot < 0.5°, σ_t < 5mm
 */
export type ValidateData = {
  source: string;
  pose_count: number;
  per_pose_residual: PerPoseResidual[];
  sigma_rot_deg: number;
  sigma_t_mm: number;
};
