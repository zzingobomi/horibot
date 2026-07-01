// applyJoints — backend TcpState.joint_names 계약 기반 매핑 단위 테스트.
//
// 회귀 잡음 — 2026-07-01 사건: `Object.entries(robot.joints)` 로 URDF 파일 순서에
// 그냥 index 매핑했던 자리 → SO-101 URDF 가 joint7..joint1 역순으로 선언되어
// backend joint1..joint6 값이 gripper→wrist 자리로 뒤집혀 렌더. 이 assert 뒤집으면
// 그 회귀가 즉시 잡힘 — jointNames prop 이 SSOT 임을 계약으로 못박음.

import { describe, expect, it, vi } from "vitest";
import { applyJoints } from "./jointMapping";
import type { URDFRobot, URDFJoint } from "urdf-loader";

/** SO-101 URDF 실 파일과 동일한 역순 (joint7..joint1) — 회귀 시나리오 재현. */
function makeMockRobot(): { robot: URDFRobot; setJointValue: ReturnType<typeof vi.fn> } {
  const setJointValue = vi.fn();
  // 파일 순서 = joint7 먼저, joint1 마지막. Object.entries 시 이 순서 대로 순회.
  const joints: Record<string, Partial<URDFJoint>> = {
    joint7: { jointType: "revolute" },
    joint6: { jointType: "revolute" },
    joint5: { jointType: "revolute" },
    joint4: { jointType: "revolute" },
    joint3: { jointType: "revolute" },
    joint2: { jointType: "revolute" },
    joint1: { jointType: "revolute" },
  };
  return {
    robot: { joints, setJointValue } as unknown as URDFRobot,
    setJointValue,
  };
}

describe("applyJoints — jointNames 기반 매핑 (URDF 파일 순서 무관)", () => {
  it("jointNames[i] 로 URDF joint 를 찾아 각 index 의 angle 을 setJointValue", () => {
    const { robot, setJointValue } = makeMockRobot();

    // backend TcpState.joint_names = motors.yaml arm prefix 순서 (joint1..joint6).
    // angles 는 backend 가 같은 index 로 보낸 rad list.
    const names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"];
    const angles = [0.11, 0.22, 0.33, 0.44, 0.55, 0.66];

    applyJoints(robot, names, angles);

    // 6 개 각 name/angle pair 로 정확히 호출 — URDF 의 역순 dict 와 무관.
    expect(setJointValue).toHaveBeenCalledTimes(6);
    names.forEach((name, i) => {
      expect(setJointValue).toHaveBeenCalledWith(name, angles[i]);
    });
  });

  it("URDF joint dict 의 순서가 역순이어도 index 매핑이 뒤집히지 않음 (회귀 원천 차단)", () => {
    // 회귀 재현: URDF 순서 (joint7..joint1) 로 index 매핑하면
    // angles[0] = 0.11 이 joint7 (gripper) 에 들어가야 하는데,
    // 정석 구현은 jointNames[0]="joint1" 로 매핑되어 joint1 이 0.11 을 받아야 함.
    const { robot, setJointValue } = makeMockRobot();
    const names = ["joint1", "joint2"];
    const angles = [0.11, 0.22];

    applyJoints(robot, names, angles);

    // joint1 이 첫 번째 angle 을 받음 (URDF dict 첫 key = joint7 이지만 무관)
    expect(setJointValue).toHaveBeenCalledWith("joint1", 0.11);
    expect(setJointValue).toHaveBeenCalledWith("joint2", 0.22);
    // joint7 (URDF dict 의 첫 key) 는 angles[0] 안 받음 — 회귀 방지 assert
    expect(setJointValue).not.toHaveBeenCalledWith("joint7", 0.11);
  });

  it("angle undefined 또는 URDF 에 없는 joint 는 skip (안전 fallback)", () => {
    const { robot, setJointValue } = makeMockRobot();
    // 6번째 이름은 URDF 에 없는 이름, 3번째는 angle undefined
    const names = ["joint1", "joint2", "joint3", "nonexistent"];
    const angles = [0.11, 0.22, undefined as unknown as number, 0.44];

    applyJoints(robot, names, angles);

    expect(setJointValue).toHaveBeenCalledWith("joint1", 0.11);
    expect(setJointValue).toHaveBeenCalledWith("joint2", 0.22);
    expect(setJointValue).not.toHaveBeenCalledWith("joint3", expect.anything());
    expect(setJointValue).not.toHaveBeenCalledWith("nonexistent", expect.anything());
  });

  it("빈 name/angle list — mount 직후 stream 도착 전 자연스러운 no-op", () => {
    const { robot, setJointValue } = makeMockRobot();
    applyJoints(robot, [], []);
    expect(setJointValue).not.toHaveBeenCalled();
  });
});
