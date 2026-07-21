// ScanPanel 자동 스캔 wire 검증 (unit).
// [자동 스캔] → WORLDSCAN_RUN (voxel 실림) / RUNNING 이면 시작 잠금·중지 활성
// (robot-busy) / 실패 사유 표시 (침묵 금지) / 월드 표시 토글 = scanStore.

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render } from "@testing-library/react";
import { bridge } from "@/api/bridge";
import { useFrameworkStore, type ServiceEntry } from "@/framework/store";
import { useScanStore } from "@/stores/scanStore";
import { RobotContext } from "@/hooks/robotContext";
import { ScanPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";
const WS_STATE = `stream/world_scan/${ROBOT_ID}/state`;

function mockBridge(dataByKey: Record<string, unknown> = {}) {
  return vi
    .spyOn(bridge, "callService")
    // @ts-expect-error — 테스트 stub
    .mockImplementation(async (key, _req, opts) => {
      const wk = bridge.serviceCacheKey(key, (opts as { robotId?: string })?.robotId);
      const entry: ServiceEntry = {
        success: true,
        message: "",
        data: dataByKey[String(key)] ?? { ok: true, accepted: true },
        timestamp: Date.now(),
        pending: false,
      };
      useFrameworkStore.getState().setServiceData(wk, entry);
      return entry;
    });
}

function renderPanel() {
  return render(
    <RobotContext.Provider value={ROBOT_ID}>
      <ScanPanel />
    </RobotContext.Provider>,
  );
}

function seedState(status: string, error: string | null = null) {
  useFrameworkStore.setState({
    topicData: {
      [WS_STATE]: {
        robot_id: ROBOT_ID,
        seq: 1,
        timestamp_unix: 0,
        status,
        task_name: "world_scan",
        current_name: "",
        current_title: "스캔 스윕",
        error,
        breakpoints: [],
      },
    },
    bridgeConnected: true,
  });
}

beforeEach(() => {
  useFrameworkStore.setState({ topicData: {}, serviceData: {}, bridgeConnected: true });
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("ScanPanel 자동 스캔", () => {
  it("자동 스캔 클릭 → WORLDSCAN_RUN 에 voxel_size 실림", async () => {
    const spy = mockBridge({ "srv/world_scan/run": { accepted: true } });
    const { getByTestId } = renderPanel();
    await act(async () => {
      fireEvent.click(getByTestId("auto-scan"));
    });
    const calls = spy.mock.calls.filter((c) => String(c[0]) === "srv/world_scan/run");
    expect(calls.length).toBe(1);
    expect((calls[0][1] as { voxel_size?: number }).voxel_size).toBe(0.002); // 기본 2mm
  });

  it("스캔 RUNNING 이면 시작 잠금 + 중지 활성 (robot-busy)", () => {
    mockBridge();
    seedState("running");
    const { getByTestId } = renderPanel();
    expect((getByTestId("auto-scan") as HTMLButtonElement).disabled).toBe(true);
    expect((getByTestId("auto-stop") as HTMLButtonElement).disabled).toBe(false);
    expect(getByTestId("auto-status").textContent).toContain("running");
  });

  it("스캔 실패 → 사유 표시 (침묵 금지)", () => {
    mockBridge();
    seedState("failed", "스캔 빌드 실패 — 정합 발산");
    const { getByTestId } = renderPanel();
    expect(getByTestId("auto-error").textContent).toContain("정합 발산");
  });

  it("월드 표시 토글 → scanStore.worldVisible 반전", async () => {
    mockBridge();
    const { getByTestId } = renderPanel();
    const before = useScanStore.getState().worldVisible;
    await act(async () => {
      fireEvent.click(getByTestId("world-visible"));
    });
    expect(useScanStore.getState().worldVisible).toBe(!before);
  });
});
