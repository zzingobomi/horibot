/**
 * Motion 도메인 type — backend pydantic (`backend/core/transport/messages/motion.py`)
 * 의 schema 를 `gen:types` 결과에서 직접 alias. drift 0.
 *
 * frontend-only 편의 type (Vec3 / Quaternion / MotionMode) 만 hand-write.
 */
import type { components } from "@/api/generated/types";

// Three.js / drei 의 Vector3 prop 호환 위해 tuple 유지. backend wire (`number[]`)
// 와의 경계는 util (`mToMmVec3` 등) 에서 narrowing.
export type Vec3 = [number, number, number];
export type Quaternion = [number, number, number, number];
export type MotionMode = "joint" | "move_tcp" | "move_j" | "move_l";

// ── backend pydantic alias (gen:types SSOT)
export type TCPPose = components["schemas"]["MotionTcpPose"];
export type TrajectoryStatus = components["schemas"]["TrajStatus"];
export type TrajectoryState = components["schemas"]["MotionTrajState"];

export type MoveTCPRequest = components["schemas"]["MoveTcpReq"];
export type MoveJRequest = components["schemas"]["MoveJReq"];
export type MoveLRequest = components["schemas"]["MoveLReq"];
export type MoveCRequest = components["schemas"]["MoveCReq"];
export type MovePRequest = components["schemas"]["MovePReq"];
