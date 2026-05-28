export type JointType = "arm" | "gripper";

export interface JointConfig {
  id: number;
  name: string;
  label: string;
  type: JointType;
}

export const JOINT_CONFIGS: readonly JointConfig[] = [
  {
    id: 1,
    name: "joint1",
    label: "Joint 1",
    type: "arm",
  },
  {
    id: 2,
    name: "joint2",
    label: "Joint 2",
    type: "arm",
  },
  {
    id: 3,
    name: "joint3",
    label: "Joint 3",
    type: "arm",
  },
  {
    id: 4,
    name: "joint4",
    label: "Joint 4",
    type: "arm",
  },
  {
    id: 5,
    name: "joint5",
    label: "Joint 5",
    type: "arm",
  },
  {
    id: 6,
    name: "gripper_joint_1",
    label: "Gripper",
    type: "gripper",
  },
];

export const ARM_JOINTS = JOINT_CONFIGS.filter((j) => j.type === "arm");

export const GRIPPER = JOINT_CONFIGS.find((j) => j.type === "gripper");

export const TCP_LINK_NAME = "end_effector_link";
