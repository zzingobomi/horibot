/**
 * Robot 도메인 type — backend `bridge/schemas.py` (RobotInfo / capabilities)
 * 의 pydantic 모델을 generated 에서 직접 alias.
 *
 * 새 capability 추가 시 backend Literal 한 번 갱신 → `pnpm gen:types` 재실행
 * → frontend 가 자동으로 새 모양 받음.
 */
import type { components } from "@/api/generated/types";

export type RobotInfo = components["schemas"]["RobotInfo"];
export type RobotBasePose = components["schemas"]["BasePoseSchema"];
export type RobotsListResponse = components["schemas"]["RobotsListResponse"];
export type RobotCapability = RobotInfo["capabilities"][number];
