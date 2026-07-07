// RobotLayer — per-robot joint 구독 회귀 가드.
//
// 2026-07-06 사건: Container 가 "focus robot 1대의 TCP stream" 을 전 robot 에
// 적용 → Tasks(focus=null) 에서 default 가 비활성 robot 이면 전원이 얼어붙고,
// N=2 에선 두 robot 이 같은 joint 로 움직임. 이 테스트는 robot 마다 자기
// stream(store 의 자기 wire key) 을 읽는지 계약으로 못박음 — 단일 stream 공유로
// 되돌리면 B 가 A 의 joints 를 받아 즉시 잡힌다.

import { describe, expect, it, vi, beforeEach } from "vitest";
import { render } from "@testing-library/react";
import { useFrameworkStore } from "@/framework/store";
import { RobotLayer } from "./RobotLayer";
import { RobotModel } from "./RobotModel";
import type { RobotInfo } from "@/api/generated/contract";

vi.mock("./RobotModel", () => ({ RobotModel: vi.fn(() => null) }));
// AxisFrame 은 R3F useFrame 사용 — Canvas 밖 render 위해 mock.
vi.mock("./AxisFrame", () => ({ AxisFrame: vi.fn(() => null) }));

const ROBOT_A: RobotInfo = {
  id: "so101_6dof_0",
  type: "so101_6dof",
  base_pose: { x: 0.4, y: 0, z: 0, yaw_deg: 0 },
  capabilities: ["move"],
  has_camera: true,
};
const ROBOT_B: RobotInfo = {
  id: "omx_f_0",
  type: "omx_f",
  base_pose: { x: 0, y: 0, z: 0, yaw_deg: 0 },
  capabilities: ["move"],
  has_camera: false,
};

function tcpState(robotId: string, joints: number[], names: string[]) {
  return {
    robot_id: robotId,
    seq: 1,
    timestamp_unix: Date.now() / 1000,
    position: [0.1, 0, 0.2],
    quaternion: [0, 0, 0, 1],
    joint_names: names,
    joints,
  };
}

const JOINTS_A = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
const JOINTS_B = [1.1, 1.2, 1.3, 1.4, 1.5];
const NAMES_A = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"];
const NAMES_B = ["joint1", "joint2", "joint3", "joint4", "joint5"];

beforeEach(() => {
  vi.mocked(RobotModel).mockClear();
  useFrameworkStore.setState({
    topicData: {
      "stream/motion/so101_6dof_0/tcp_state": tcpState("so101_6dof_0", JOINTS_A, NAMES_A),
      "stream/motion/omx_f_0/tcp_state": tcpState("omx_f_0", JOINTS_B, NAMES_B),
    },
    serviceData: {},
    bridgeConnected: true,
  });
});

function propsFor(robotType: string) {
  const call = vi
    .mocked(RobotModel)
    .mock.calls.find(([p]) => p.robotType === robotType);
  expect(call).toBeTruthy();
  return call![0];
}

describe("RobotLayer — robot 마다 자기 TCP stream 구독", () => {
  it("focus=null (Tasks): 두 robot 이 각자 자기 joints 를 받음", () => {
    render(<RobotLayer robots={[ROBOT_A, ROBOT_B]} focusId={null} />);

    const a = propsFor("so101_6dof");
    const b = propsFor("omx_f");
    expect(a.jointAngles).toEqual(JOINTS_A);
    expect(a.jointNames).toEqual(NAMES_A);
    // 회귀 핵심 — B 는 A 의 stream 이 아니라 자기 stream (단일 stream 공유면 여기서 깨짐)
    expect(b.jointAngles).toEqual(JOINTS_B);
    expect(b.jointNames).toEqual(NAMES_B);
    // focus=null → 모두 불투명
    expect(a.opacity).toBe(1.0);
    expect(b.opacity).toBe(1.0);
  });

  it("focus 지정: non-focus 도 자기 실 상태 (dim 만 다름)", () => {
    render(<RobotLayer robots={[ROBOT_A, ROBOT_B]} focusId="so101_6dof_0" />);

    const a = propsFor("so101_6dof");
    const b = propsFor("omx_f");
    expect(a.opacity).toBe(1.0);
    expect(b.opacity).toBe(0.25); // dim
    expect(b.jointAngles).toEqual(JOINTS_B); // dim 이어도 자기 joint (home 고정 아님)
  });

  it("gripper 필드 있으면 arm 뒤에 append (URDF open/close 시각화)", () => {
    // gripper 는 arm(joints)과 분리된 별도 필드 → arm 뒤에 이름+rad append.
    // 이름 기반 URDF 매핑이라 joint7 이 열림/닫힘으로 렌더. joints(arm)에 안 섞임
    // (섞이면 waypoint/MoveJ dof 회귀).
    useFrameworkStore.setState({
      topicData: {
        "stream/motion/so101_6dof_0/tcp_state": {
          ...tcpState("so101_6dof_0", JOINTS_A, NAMES_A),
          gripper_joint_name: "joint7",
          gripper_rad: 0.9,
        },
      },
      serviceData: {},
      bridgeConnected: true,
    });
    render(<RobotLayer robots={[ROBOT_A]} focusId={null} />);

    const a = propsFor("so101_6dof");
    expect(a.jointNames).toEqual([...NAMES_A, "joint7"]);
    expect(a.jointAngles).toEqual([...JOINTS_A, 0.9]);
  });

  it("stream 미도착 robot 은 빈 배열 (URDF 기본 pose 안전 fallback)", () => {
    useFrameworkStore.setState({ topicData: {}, serviceData: {}, bridgeConnected: true });
    render(<RobotLayer robots={[ROBOT_A]} focusId={null} />);

    const a = propsFor("so101_6dof");
    expect(a.jointAngles).toEqual([]);
    expect(a.jointNames).toEqual([]);
  });
});
