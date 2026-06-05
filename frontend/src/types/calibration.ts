/**
 * Calibration 도메인 type — backend `/calibration/results` 응답 schema 의
 * `gen:types` 산출물 alias. SSOT (backend `bridge/schemas.py`).
 *
 * 4종 산출물 + intrinsic 각각의 적용 메커니즘은 CLAUDE.md "캘리브레이션
 * 4종 산출물 + intrinsic" 표 참조. frontend 는 URDF / Detector 시각화 자리에서만.
 */
import type { components } from "@/api/generated/types";

export type IntrinsicSchema = components["schemas"]["IntrinsicSchema"];
export type HandEyeSchema = components["schemas"]["HandEyeSchema"];
export type CalibrationResults = components["schemas"]["CalibrationResults"];
