// RobotFrame — scene contribution 의 좌표 primitive 계약.
// robot 미발견(로딩) = null 대기 / robotId 미해결(배선 버그) = throw / context 기본값.

import { describe, it, expect, vi } from "vitest";
import { render } from "@testing-library/react";
import { RobotContext } from "@/hooks/robotContext";
import { RobotFrame } from "./RobotFrame";

let mockRobots: { id: string; type: string; base_pose?: object }[] = [];
vi.mock("@/hooks/useRobots", () => ({
  useRobots: () => ({ robots: mockRobots, loading: false, error: null }),
}));

describe("RobotFrame", () => {
  it("robotId 명시 + robot 존재 → children 렌더", () => {
    mockRobots = [{ id: "a", type: "t", base_pose: { x: 0.4, y: 0, z: 0, yaw_deg: 0 } }];
    const { getByTestId } = render(
      <RobotFrame robotId="a">
        <div data-testid="child" />
      </RobotFrame>,
    );
    expect(getByTestId("child")).toBeTruthy();
  });

  it("robot 이 목록에 없음(로딩/transient) → null (children 미렌더)", () => {
    mockRobots = [];
    const { queryByTestId } = render(
      <RobotFrame robotId="a">
        <div data-testid="child" />
      </RobotFrame>,
    );
    expect(queryByTestId("child")).toBeNull();
  });

  it("robotId 생략 시 RobotContext 에서 (scenePart 안 사용 형태)", () => {
    mockRobots = [{ id: "ctx-bot", type: "t" }];
    const { getByTestId } = render(
      <RobotContext.Provider value="ctx-bot">
        <RobotFrame>
          <div data-testid="child" />
        </RobotFrame>
      </RobotContext.Provider>,
    );
    expect(getByTestId("child")).toBeTruthy();
  });

  it("robotId 도 context 도 없음 = 배선 버그 → throw (useRobotId 와 같은 계약)", () => {
    mockRobots = [];
    const silence = vi.spyOn(console, "error").mockImplementation(() => {});
    expect(() =>
      render(
        <RobotFrame>
          <div />
        </RobotFrame>,
      ),
    ).toThrow(/robotId/);
    silence.mockRestore();
  });
});
