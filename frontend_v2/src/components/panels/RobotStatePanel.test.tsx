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

    // 초기 torque state — Motor.Stream.STATE 5Hz 발행 후 store 갱신 (torque_enabled=true).
    // (backend_v2 결정 B — state ≠ event, torque 현재값은 stream 소유.)
    act(() => {
      useFrameworkStore
        .getState()
        .setTopicData(`stream/motor/${ROBOT_ID}/state`, {
          robot_id: ROBOT_ID,
          seq: 1,
          timestamp_unix: Date.now() / 1000,
          torque_enabled: true,
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

  // 회귀 잡음 — 2026-07-01 사건: torque 초기값 얻는 자리가 event 였을 때 chicken-and-egg
  // (event 는 set_torque 순간에만 발행 → 초기 state unknown → 버튼 영구 disabled).
  // 이 assert 뒤집으면 그 회귀가 잡힘 — "MotorState stream 도착 → 버튼 활성화" 가 계약.
  it("mount 직후 MotorState 도착 시 버튼 즉시 활성화 (chicken-and-egg 회귀 차단)", async () => {
    vi.spyOn(bridge, "callService").mockResolvedValue({
      success: true,
      message: "",
      data: { motors: [{ id: 1, kind: "joint" as const }] } as never,
    });

    const { findByRole } = renderPanel();

    // 초기 = state 도착 전 — 버튼 disabled + label ambiguous
    const btnBeforeState = await findByRole("button", { name: /torque/i });
    expect(btnBeforeState).toBeDisabled();
    expect(btnBeforeState.textContent).toMatch(/torque on/); // torqueEnabled=null → default label

    // Motor.Stream.STATE 첫 frame 도착 — 실 wire 는 5Hz, mount 후 ~200ms 안 자연 도착
    act(() => {
      useFrameworkStore
        .getState()
        .setTopicData(`stream/motor/${ROBOT_ID}/state`, {
          robot_id: ROBOT_ID,
          seq: 0,
          timestamp_unix: Date.now() / 1000,
          torque_enabled: false,
        });
    });

    // state 도착 순간 — 버튼 활성화 + 값 반영
    await waitFor(() => {
      const btn = btnBeforeState;
      expect(btn).not.toBeDisabled();
      expect(btn.textContent).toMatch(/torque on/); // enabled=false → "torque on" 라벨
    });

    // torque_enabled=true 로 갱신 — 라벨/badge 반응 검증
    act(() => {
      useFrameworkStore
        .getState()
        .setTopicData(`stream/motor/${ROBOT_ID}/state`, {
          robot_id: ROBOT_ID,
          seq: 1,
          timestamp_unix: Date.now() / 1000,
          torque_enabled: true,
        });
    });
    await waitFor(() => {
      expect(btnBeforeState.textContent).toMatch(/torque off/);
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
