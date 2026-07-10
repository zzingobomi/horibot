// Cameras — 카메라 씬 객체 계약.
// rgbd robot 만 카메라 파생 / frustum·cloud 각자 gate / cloud 는 자기 robot 의
// cloud topic 구독. (렌더는 카메라당 한 번 — 패널 수와 무관, 소유권 모델)

import { describe, it, expect, vi, beforeEach } from "vitest";
import { render } from "@testing-library/react";
import { bridge } from "@/api/bridge";
import { useFrameworkStore } from "@/framework/store";
import { useCameraStore } from "@/stores/cameraStore";
import { useScanStore } from "@/stores/scanStore";
import { Cameras } from "./Cameras";
import type { RobotInfo } from "@/api/generated/contract";

// AxisFrame(useFrame — Canvas 밖 불가) 회피 — Frame primitive mock (Robots.test 와 동일 사유)
vi.mock("../shared/primitives", () => ({ Frame: () => null }));

const OMX: RobotInfo = {
  id: "omx_f_0",
  type: "omx_f",
  capabilities: ["move", "calibrate"],
  base_pose: { x: 0, y: 0, z: 0, yaw_deg: 0 },
};
const SO101: RobotInfo = {
  id: "so101_6dof_0",
  type: "so101_6dof",
  capabilities: ["move", "calibrate", "rgbd"],
  base_pose: { x: 0.4, y: 0, z: 0, yaw_deg: 0 },
};

vi.mock("@/hooks/useRobots", () => ({
  useRobots: () => ({ robots: [OMX, SO101], loading: false, error: null }),
}));

function seedTcp(...robotIds: string[]) {
  useFrameworkStore.setState({
    topicData: Object.fromEntries(
      robotIds.map((robotId) => [
        `stream/motion/${robotId}/tcp_state`,
        {
          robot_id: robotId,
          seq: 1,
          timestamp_unix: Date.now() / 1000,
          position: [0.1, 0, 0.2],
          quaternion: [0, 0, 0, 1],
          joint_names: [],
          joints: [],
        },
      ]),
    ),
    serviceData: {},
    bridgeConnected: false, // useMirror fetch 억제 — handEye identity 경로
  });
}

beforeEach(() => {
  vi.restoreAllMocks();
  useCameraStore.setState({ showFrustum: false });
  useScanStore.setState({ liveEnabled: false });
  useFrameworkStore.setState({ topicData: {}, serviceData: {}, bridgeConnected: false });
});

describe("Cameras — 카메라 씬 객체", () => {
  it("showFrustum=true → rgbd robot 카메라에만 frustum 1개 (omx 는 카메라 없음)", () => {
    seedTcp(SO101.id, OMX.id); // omx 에 tcp 가 있어도 rgbd 없으면 카메라 미파생
    useCameraStore.setState({ showFrustum: true });
    const { container } = render(<Cameras robots={[OMX, SO101]} focusId={null} />);
    expect(container.querySelectorAll("linesegments")).toHaveLength(1);
  });

  it("showFrustum=false && liveEnabled=false → 아무것도 렌더 안 함", () => {
    seedTcp(SO101.id);
    const { container } = render(<Cameras robots={[OMX, SO101]} focusId={null} />);
    expect(container.querySelectorAll("linesegments")).toHaveLength(0);
    expect(container.querySelectorAll("points")).toHaveLength(0);
  });

  it("tcp stream 없음 → pose 미상, frustum 켜도 렌더 보류", () => {
    useCameraStore.setState({ showFrustum: true });
    const { container } = render(<Cameras robots={[SO101]} focusId={null} />);
    expect(container.querySelectorAll("linesegments")).toHaveLength(0);
  });

  it("liveEnabled=true → 자기 robot 의 cloud topic 구독 + points 렌더", () => {
    seedTcp(SO101.id);
    useScanStore.setState({ liveEnabled: true });
    const subSpy = vi
      .spyOn(bridge, "subscribeBinary")
      .mockReturnValue(() => undefined);
    const { container } = render(<Cameras robots={[SO101]} focusId={null} />);
    expect(container.querySelectorAll("points")).toHaveLength(1);
    expect(subSpy).toHaveBeenCalledTimes(1);
    expect(String(subSpy.mock.calls[0][0])).toContain(SO101.id);
  });
});
