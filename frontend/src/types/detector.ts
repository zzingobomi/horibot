/**
 * Detector 도메인 type — backend pydantic schema 직접 alias.
 */
import type { components } from "@/api/generated/types";

export type Detection = components["schemas"]["YoloDetection"];
export type GroundedResult = components["schemas"]["GroundedDetectionResult"];
