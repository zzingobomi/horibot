/**
 * Motor 도메인 type — backend pydantic (`backend/core/transport/messages/motor.py`)
 * 의 schema 를 `gen:types` 결과에서 직접 alias. drift 0.
 */
import type { components } from "@/api/generated/types";

export type Joint = components["schemas"]["MotorJoint"];
export type JointState = components["schemas"]["MotorJointState"];
export type JointCmd = components["schemas"]["MotorCmdJoint"];
export type MotorConfig = components["schemas"]["MotorConfigItem"];
