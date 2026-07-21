// ScanGrowth — world_scan 성장 점군의 scene3d 스트림 배선 계약.
// 2026-07-21 홈-검증 실사고: SET_STREAM enable 배선이 없어 스캔 내내 cloud
// frame 0개 = 성장 점군이 아예 안 떴다. 스캔 시작=enable / 끝=disable,
// 단 LivePointCloudPanel 이 켜 둔 상태(scanStore.liveEnabled)면 끄지 않는다.

import { describe, it, expect, vi, beforeEach } from "vitest";
import { render, act } from "@testing-library/react";
import { bridge } from "@/api/bridge";
import { useFrameworkStore } from "@/framework/store";
import { useScanStore } from "@/stores/scanStore";
import { ServiceKey } from "@/api/generated/contract";
import { ScanGrowth } from "./ScanGrowth";
import type { RobotInfo } from "@/api/generated/contract";

const SO101: RobotInfo = {
  id: "so101_6dof_0",
  type: "so101_6dof",
  capabilities: ["move", "calibrate", "rgbd"],
  base_pose: { x: 0, y: 0, z: 0, yaw_deg: 0 },
  has_camera: true,
};

const WS_KEY = `stream/world_scan/${SO101.id}/state`;

function seedScanState(status: string, seq = 1) {
  useFrameworkStore.setState((s) => ({
    topicData: {
      ...s.topicData,
      [WS_KEY]: {
        robot_id: SO101.id,
        seq,
        timestamp_unix: Date.now() / 1000,
        status,
        task_name: "world_scan",
        current_name: "",
        current_title: "",
        error: "",
        breakpoints: [],
      },
    },
  }));
}

function setStreamCalls(spy: ReturnType<typeof vi.spyOn>) {
  return spy.mock.calls.filter(
    (c: unknown[]) => c[0] === ServiceKey.SCENE3D_SET_STREAM,
  ) as unknown as [string, { enabled: boolean; voxel_size?: number }][];
}

beforeEach(() => {
  vi.restoreAllMocks();
  useScanStore.setState({ liveEnabled: {} });
  useFrameworkStore.setState({
    topicData: {},
    serviceData: {},
    bridgeConnected: false, // useMirror fetch 억제
  });
});

describe("ScanGrowth — scene3d 스트림 배선", () => {
  it("스캔 running 이 되면 SET_STREAM enable (voxel 동봉) — 배선 부재 회귀망", () => {
    const call = vi
      .spyOn(bridge, "callService")
      .mockResolvedValue(undefined as never);
    vi.spyOn(bridge, "subscribeBinary").mockReturnValue(() => {});
    seedScanState("running");
    render(<ScanGrowth robots={[SO101]} focusId={SO101.id} />);
    const calls = setStreamCalls(call);
    expect(calls).toHaveLength(1);
    expect(calls[0][1]).toMatchObject({
      robot_id: SO101.id,
      enabled: true,
      voxel_size: useScanStore.getState().voxelSize,
    });
  });

  it("스캔 종료 시 disable — 단, 우리가 켠 것만 (패널 미사용 상태)", () => {
    const call = vi
      .spyOn(bridge, "callService")
      .mockResolvedValue(undefined as never);
    vi.spyOn(bridge, "subscribeBinary").mockReturnValue(() => {});
    seedScanState("running");
    render(<ScanGrowth robots={[SO101]} focusId={SO101.id} />);
    act(() => seedScanState("success", 2));
    const calls = setStreamCalls(call);
    expect(calls).toHaveLength(2);
    expect(calls[1][1]).toMatchObject({ robot_id: SO101.id, enabled: false });
  });

  it("LivePointCloudPanel 이 켜 둔 상태면 종료해도 끄지 않는다 (소유 존중)", () => {
    const call = vi
      .spyOn(bridge, "callService")
      .mockResolvedValue(undefined as never);
    vi.spyOn(bridge, "subscribeBinary").mockReturnValue(() => {});
    useScanStore.getState().setLiveEnabled(SO101.id, true);
    seedScanState("running");
    render(<ScanGrowth robots={[SO101]} focusId={SO101.id} />);
    act(() => seedScanState("success", 2));
    const calls = setStreamCalls(call);
    expect(calls).toHaveLength(1); // enable 만 — disable 없음
    expect(calls[0][1]).toMatchObject({ enabled: true });
  });
});
