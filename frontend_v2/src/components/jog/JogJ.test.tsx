// frontend_v2.md §12.2 JogJ — 3 invariant.
// JogTcp 의 동일 invariant 는 Step F5 의 L4 Playwright 가 검증 (mock backend e2e).

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render, waitFor } from "@testing-library/react";
import { bridge } from "@/api/bridge";
import { _resetCapabilityCache } from "@/framework/capability";
import { useFrameworkStore } from "@/framework/store";
import { Topic } from "@/api/generated/contract";
import { JogJ } from "./JogJ";

const ROBOT_ID = "so101_6dof_0";
const MOTOR_TOPOLOGY = {
  motors: [
    { id: 1, kind: "joint" as const },
    { id: 2, kind: "joint" as const },
    { id: 3, kind: "joint" as const },
    { id: 4, kind: "joint" as const },
    { id: 5, kind: "joint" as const },
    { id: 6, kind: "joint" as const },
    { id: 7, kind: "gripper" as const },
  ],
};

beforeEach(() => {
  _resetCapabilityCache();
  useFrameworkStore.setState({
    topicData: {},
    serviceData: {},
    // useCapability 가 connected dep — test 에선 connected=true 가정
    bridgeConnected: true,
  });
  // useCapability 가 호출하는 bridge.callService — MotorTopology 즉시 resolve
  vi.spyOn(bridge, "callService").mockResolvedValue({
    success: true,
    message: "",
    data: MOTOR_TOPOLOGY as never,
  });
});

afterEach(() => {
  vi.useRealTimers();
  vi.restoreAllMocks();
});

describe("JogJ", () => {
  // spec frontend_v2.md §12.2 — invariant: button hold 시 50Hz publish + payload.robot_id
  it("50Hz interval publish + payload.robot_id + velocities 박힘", async () => {
    const publishSpy = vi.spyOn(bridge, "publish").mockImplementation(() => {});

    const { findByText } = render(<JogJ robotId={ROBOT_ID} />);
    // useCapability 가 motors 박을 때까지 — J1 버튼이 뜨면 ready
    await findByText("J1");

    vi.useFakeTimers();
    // J1 의 + 버튼 = 두 번째 button (− 다음 +). 단 J1 row 안 "+" text 박혀있음.
    const plusButtons = document.querySelectorAll("button");
    // J1 row 의 "+" button (J1 의 두 번째 button)
    const j1Plus = Array.from(plusButtons).find(
      (b) => b.textContent === "+",
    );
    expect(j1Plus).toBeTruthy();

    await act(async () => {
      fireEvent.pointerDown(j1Plus!);
    });

    // 100ms = 50Hz × 5 publish (20ms interval × 5)
    await act(async () => {
      await vi.advanceTimersByTimeAsync(100);
    });

    expect(publishSpy.mock.calls.length).toBeGreaterThanOrEqual(4);
    // payload 검증
    const [topic, data, robotIdArg] = publishSpy.mock.calls[0];
    expect(topic).toBe(Topic.MOTION_JOG_J);
    expect(robotIdArg).toBe(ROBOT_ID);
    expect((data as { robot_id: string }).robot_id).toBe(ROBOT_ID);
    expect((data as { velocities: number[] }).velocities).toHaveLength(6);
    // J1 (idx 0) 에 + 방향 → positive
    expect((data as { velocities: number[] }).velocities[0]).toBeGreaterThan(0);
    // 다른 joint 자리 = 0
    expect((data as { velocities: number[] }).velocities[1]).toBe(0);
  });

  // spec frontend_v2.md §12.2 — invariant: pointerUp → interval clear → publish 안 함
  it("pointerUp → publish 중단", async () => {
    const publishSpy = vi.spyOn(bridge, "publish").mockImplementation(() => {});

    const { findByText } = render(<JogJ robotId={ROBOT_ID} />);
    await findByText("J1");

    vi.useFakeTimers();
    const j1Plus = Array.from(document.querySelectorAll("button")).find(
      (b) => b.textContent === "+",
    )!;

    await act(async () => {
      fireEvent.pointerDown(j1Plus);
    });
    await act(async () => {
      await vi.advanceTimersByTimeAsync(60);
    });
    const countBefore = publishSpy.mock.calls.length;
    expect(countBefore).toBeGreaterThan(0);

    // pointerUp on window
    await act(async () => {
      window.dispatchEvent(new Event("pointerup"));
    });
    await act(async () => {
      await vi.advanceTimersByTimeAsync(100);
    });

    // pointerUp 후 publish 더 이상 추가 안 됨
    expect(publishSpy.mock.calls.length).toBe(countBefore);
  });

  // spec frontend_v2.md §12.2 — invariant: window blur → deadman, publish 중단
  it("window blur — deadman 박힘 (publish 중단)", async () => {
    const publishSpy = vi.spyOn(bridge, "publish").mockImplementation(() => {});

    const { findByText } = render(<JogJ robotId={ROBOT_ID} />);
    await findByText("J1");

    vi.useFakeTimers();
    const j1Plus = Array.from(document.querySelectorAll("button")).find(
      (b) => b.textContent === "+",
    )!;

    await act(async () => {
      fireEvent.pointerDown(j1Plus);
    });
    await act(async () => {
      await vi.advanceTimersByTimeAsync(60);
    });
    const countBefore = publishSpy.mock.calls.length;

    await act(async () => {
      window.dispatchEvent(new Event("blur"));
    });
    await act(async () => {
      await vi.advanceTimersByTimeAsync(100);
    });

    expect(publishSpy.mock.calls.length).toBe(countBefore);
  });
});
