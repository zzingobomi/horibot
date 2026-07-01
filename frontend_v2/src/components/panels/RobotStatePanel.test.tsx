// frontend_v2.md §12.2 RobotStatePanel — 2 invariant.
// 패널이 useParams 로 robotId 를 self-read 하므로 MemoryRouter + /robots/:id
// route 로 감싸 렌더 (router 의존은 패널에서 끝 — §2.3).

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render, waitFor } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { bridge } from "@/api/bridge";
import { _resetCapabilityCache } from "@/framework/capability";
import { useFrameworkStore } from "@/framework/store";
import { RobotStatePanel } from "./RobotStatePanel";

const ROBOT_ID = "so101_6dof_0";

function renderPanel() {
  return render(
    <MemoryRouter initialEntries={[`/robots/${ROBOT_ID}`]}>
      <Routes>
        <Route path="/robots/:id" element={<RobotStatePanel />} />
      </Routes>
    </MemoryRouter>,
  );
}

beforeEach(() => {
  _resetCapabilityCache();
  useFrameworkStore.setState({
    topicData: {},
    serviceData: {},
    bridgeConnected: true,
  });
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("RobotStatePanel", () => {
  // spec frontend_v2.md §12.2 — invariant: torque toggle → setTorque service call
  it("torque toggle 시 Motor.Service.SET_TORQUE call (enabled 반전)", async () => {
    // Capability fetch (topology) — boot 1회. SET_TORQUE 호출은 별도 mock.
    const callSpy = vi.spyOn(bridge, "callService").mockImplementation((key) =>
      Promise.resolve({
        success: true,
        message: "",
        data: String(key).includes("topology")
          ? ({ motors: [{ id: 1, kind: "joint" as const }] } as never)
          : ({ ok: true } as never),
      }),
    );

    // 초기 torque state — TORQUE_CHANGED event 도착 후 store 갱신 (enabled=true)
    act(() => {
      useFrameworkStore
        .getState()
        .setTopicData(`event/motor/${ROBOT_ID}/torque_changed`, {
          robot_id: ROBOT_ID,
          enabled: true,
        });
    });

    const { findByText } = renderPanel();

    // capability 받은 후 button render 됨
    const toggleBtn = await findByText("torque off");

    await act(async () => {
      fireEvent.click(toggleBtn);
    });

    // setTorque.call → bridge.callService(SET_TORQUE, {enabled: false}) — torque on → off
    await waitFor(() => {
      const torqueCalls = callSpy.mock.calls.filter((c) =>
        String(c[0]).includes("set_torque"),
      );
      expect(torqueCalls.length).toBeGreaterThan(0);
      const [, reqData] = torqueCalls[0];
      expect((reqData as { enabled: boolean }).enabled).toBe(false);
    });
  });

  // spec frontend_v2.md §12.2 — invariant: TCP stream stale → "stale" badge 표시
  it("Motion.Stream.TCP_STATE 의 timestamp_unix old → stale badge 표시", async () => {
    vi.spyOn(bridge, "callService").mockResolvedValue({
      success: true,
      message: "",
      data: { motors: [{ id: 1, kind: "joint" as const }] } as never,
    });

    const oldTs = (Date.now() - 700) / 1000;
    act(() => {
      useFrameworkStore
        .getState()
        .setTopicData(`stream/motion/${ROBOT_ID}/tcp_state`, {
          robot_id: ROBOT_ID,
          seq: 1,
          timestamp_unix: oldTs,
          position: [0, 0, 0.3],
          quaternion: [0, 0, 0, 1],
          joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
          joints: [0, 0, 0, 0, 0, 0],
        });
    });

    const { findByText } = renderPanel();

    // useStream 가 lagMs > 500 → stale=true → badge "stale 700ms" 등 표시
    await findByText(/stale/);
  });
});
