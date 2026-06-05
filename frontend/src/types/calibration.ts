/**
 * Calibration 결과 type — backend `/calibration/results` HTTP endpoint 응답.
 *
 * 4종 산출물 + intrinsic — 각각 적용 메커니즘이 다름 (CLAUDE.md "캘리브레이션
 * 4종 산출물 + intrinsic" 표 참조). frontend 는 URDF / Detector 시각화 자리에서만.
 */

export interface IntrinsicData {
  camera_matrix: number[][]; // 3x3
  dist_coeffs: number[][]; // 1xN
  image_size?: number[]; // [w, h]
}

export interface HandEyeData {
  R: number[][]; // 3x3 rotation matrix
  t: number[][]; // 3x1 translation [m]
  available_keys: string[];
}

export interface JointOffsetEntry {
  motor_id: number;
  offset_rad: number;
}

export interface CalibrationResults {
  intrinsic?: IntrinsicData;
  hand_eye?: HandEyeData;
  joint_offsets?: JointOffsetEntry[];
  intrinsic_error?: string;
  hand_eye_error?: string;
}
