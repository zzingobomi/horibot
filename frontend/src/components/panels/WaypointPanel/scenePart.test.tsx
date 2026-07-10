// WaypointScenePart — ghost 미리보기 계약.
// store preview → RobotModel(반투명+tint, waypoint joints) / 없으면 null.

import { describe, it, expect, vi, beforeEach } from "vitest";
import { render } from "@testing-library/react";
import { RobotProvider } from "@/components/shared/robotOwnership";
import { RobotModel } from "@/components/scene/objects/RobotModel";
import { useWaypointStore } from "@/stores/waypointStore";
import { WaypointScenePart } from "./scenePart";

vi.mock("@/components/scene/objects/RobotModel", () => ({
  RobotModel: vi.fn(() => null),
}));

vi.mock("@/hooks/useRobots", () => ({
  useRobots: () => ({
    robots: [
      {
        id: "so101_6dof_0",
        type: "so101_6dof",
        base_pose: { x: 0.4, y: 0, z: 0, yaw_deg: 0 },
        capabilities: ["move"],
      },
    ],
    loading: false,
    error: null,
  }),
}));

const ROBOT_ID = "so101_6dof_0";

function renderPart() {
  return render(
    <RobotProvider robotId={ROBOT_ID}>
      <WaypointScenePart />
    </RobotProvider>,
  );
}

beforeEach(() => {
  vi.mocked(RobotModel).mockClear();
  useWaypointStore.setState({ previews: {} });
});

describe("WaypointScenePart", () => {
  it("preview 없음 → 아무것도 렌더 안 함", () => {
    renderPart();
    expect(vi.mocked(RobotModel)).not.toHaveBeenCalled();
  });

  it("preview 있음 → RobotModel ghost (waypoint joints + 반투명 + tint)", () => {
    useWaypointStore.getState().setPreview(ROBOT_ID, {
      waypointId: 7,
      name: "search_left",
      jointNames: ["joint1", "joint2"],
      jointAngles: [0.5, -0.3],
    });
    renderPart();
    const call = vi.mocked(RobotModel).mock.calls[0][0];
    expect(call.jointNames).toEqual(["joint1", "joint2"]);
    expect(call.jointAngles).toEqual([0.5, -0.3]);
    expect(call.opacity).toBeLessThan(1); // ghost
    expect(call.tint).toBeTruthy(); // 실 로봇과 구분
    expect(call.robotType).toBe("so101_6dof");
  });

  it("다른 robot 의 preview 는 무시 (per-robot 바인딩)", () => {
    useWaypointStore.getState().setPreview("other_robot", {
      waypointId: 1,
      name: "x",
      jointNames: ["j1"],
      jointAngles: [0],
    });
    renderPart();
    expect(vi.mocked(RobotModel)).not.toHaveBeenCalled();
  });
});
