// Cameras — 카메라 씬 객체 계약.
// has_camera robot 마다 카메라 파생 (rgbd 아님 — omx USB 웹캠도 hand_eye 캘 대상) /
// frustum 은 per-robot 토글 / cloud 는 rgbd robot 만 + 자기 robot 의 cloud topic
// 구독. (렌더는 카메라당 한 번 — 패널 수와 무관, 소유권 모델)

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
  has_camera: true, // USB 웹캠 — rgbd 아님
};
const SO101: RobotInfo = {
  id: "so101_6dof_0",
  type: "so101_6dof",
  capabilities: ["move", "calibrate", "rgbd"],
  base_pose: { x: 0.4, y: 0, z: 0, yaw_deg: 0 },
  has_camera: true, // D405
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
  useCameraStore.setState({ frustum: {} });
  useScanStore.setState({ liveEnabled: {} });
  useFrameworkStore.setState({ topicData: {}, serviceData: {}, bridgeConnected: false });
});

describe("Cameras — 카메라 씬 객체", () => {
  it("frustum 은 per-robot — omx 만 켜면 omx 카메라 1개만 (so101 안 뜸)", () => {
    // 회귀망: 옛 전역 bool 은 omx 캘 패널 [시야] 가 so101 frustum 을 띄웠음
    seedTcp(SO101.id, OMX.id);
    useCameraStore.getState().setFrustum(OMX.id, true);
    const { container } = render(<Cameras robots={[OMX, SO101]} focusId={null} />);
    expect(container.querySelectorAll("linesegments")).toHaveLength(1);
  });

  it("두 robot 다 켜면 frustum 2개 — omx(웹캠) 도 has_camera 라 카메라 파생", () => {
    seedTcp(SO101.id, OMX.id);
    useCameraStore.getState().setFrustum(OMX.id, true);
    useCameraStore.getState().setFrustum(SO101.id, true);
    const { container } = render(<Cameras robots={[OMX, SO101]} focusId={null} />);
    expect(container.querySelectorAll("linesegments")).toHaveLength(2);
  });

  it("frustum off && liveEnabled=false → 아무것도 렌더 안 함", () => {
    seedTcp(SO101.id);
    const { container } = render(<Cameras robots={[OMX, SO101]} focusId={null} />);
    expect(container.querySelectorAll("linesegments")).toHaveLength(0);
    expect(container.querySelectorAll("points")).toHaveLength(0);
  });

  it("tcp stream 없음 → pose 미상, frustum 켜도 렌더 보류", () => {
    useCameraStore.getState().setFrustum(SO101.id, true);
    const { container } = render(<Cameras robots={[SO101]} focusId={null} />);
    expect(container.querySelectorAll("linesegments")).toHaveLength(0);
  });

  it("liveEnabled=true → rgbd robot 만 cloud (omx 웹캠은 구독 X)", () => {
    seedTcp(SO101.id, OMX.id);
    // per-robot — 둘 다 켜도 rgbd(so101) 만 cloud (omx 는 depth 없음)
    useScanStore.getState().setLiveEnabled(SO101.id, true);
    useScanStore.getState().setLiveEnabled(OMX.id, true);
    const subSpy = vi
      .spyOn(bridge, "subscribeBinary")
      .mockReturnValue(() => undefined);
    const { container } = render(<Cameras robots={[OMX, SO101]} focusId={null} />);
    expect(container.querySelectorAll("points")).toHaveLength(1);
    expect(subSpy).toHaveBeenCalledTimes(1);
    expect(String(subSpy.mock.calls[0][0])).toContain(SO101.id);
  });
});
