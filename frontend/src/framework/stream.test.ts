// frontend_v2.md §12.2 useStream — 5 invariant.

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, renderHook } from "@testing-library/react";
import { useFrameworkStore } from "./store";
import { useStream } from "./stream";

const WIRE = "stream/motion/so101_6dof_0/tcp_state";

beforeEach(() => {
  useFrameworkStore.setState({
    topicData: {},
    serviceData: {},
    bridgeConnected: false,
  });
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("useStream", () => {
  // spec frontend_v2.md §12.2 — invariant: seq monotonic (1, 2, 3) → outOfOrderCount=0
  it("seq monotonic 정상 — outOfOrderCount=0 유지", () => {
    const { result } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useStream(WIRE as any, { robotId: "so101_6dof_0" }),
    );

    for (const seq of [1, 2, 3]) {
      act(() => {
        useFrameworkStore
          .getState()
          .setTopicData(WIRE, { robot_id: "so101_6dof_0", seq, timestamp_unix: Date.now() / 1000 });
      });
    }

    expect(result.current.outOfOrderCount).toBe(0);
    expect(result.current.seq).toBe(3);
  });

  // spec frontend_v2.md §12.2 — invariant: seq 역행 → outOfOrderCount 증가 + console.warn
  it("seq 역행 — outOfOrderCount 증가 + console.warn 박음", () => {
    const warnSpy = vi.spyOn(console, "warn").mockImplementation(() => {});
    const { result } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useStream(WIRE as any, { robotId: "so101_6dof_0" }),
    );

    for (const seq of [1, 2, 1]) {
      act(() => {
        useFrameworkStore
          .getState()
          .setTopicData(WIRE, { robot_id: "so101_6dof_0", seq, timestamp_unix: Date.now() / 1000 });
      });
    }

    expect(result.current.outOfOrderCount).toBe(1);
    expect(warnSpy).toHaveBeenCalledWith(
      expect.stringMatching(/seq 역행: 2 -> 1/),
    );
  });

  // spec frontend_v2.md §12.2 — invariant: timestamp_unix old → lagMs ≈ delta
  it("timestamp_unix lag detect — lagMs ≈ delta", () => {
    const oldTs = (Date.now() - 1000) / 1000;
    const { result } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useStream(WIRE as any, { robotId: "so101_6dof_0" }),
    );

    act(() => {
      useFrameworkStore
        .getState()
        .setTopicData(WIRE, { robot_id: "so101_6dof_0", seq: 1, timestamp_unix: oldTs });
    });

    // 1000ms 차이 — ±50ms 범위 박혀있을 자리 (test 실행 시간)
    expect(result.current.lagMs).toBeGreaterThanOrEqual(950);
    expect(result.current.lagMs).toBeLessThanOrEqual(1100);
  });

  // spec frontend_v2.md §12.2 — invariant: stale = lagMs > staleMs
  it("stale flag — lagMs > staleMs (default 500) → stale=true", () => {
    const oldTs = (Date.now() - 700) / 1000;
    const { result } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useStream(WIRE as any, { robotId: "so101_6dof_0" }),
    );

    act(() => {
      useFrameworkStore
        .getState()
        .setTopicData(WIRE, { robot_id: "so101_6dof_0", seq: 1, timestamp_unix: oldTs });
    });

    expect(result.current.stale).toBe(true);
  });

  // spec frontend_v2.md §12.2 — invariant: seq field 박지 X → graceful (warn X)
  it("seq field 박지 X — graceful (outOfOrderCount=0, lagMs=0)", () => {
    const warnSpy = vi.spyOn(console, "warn").mockImplementation(() => {});
    const { result } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useStream(WIRE as any, { robotId: "so101_6dof_0" }),
    );

    act(() => {
      // seq / timestamp_unix 박지 X 인 payload
      useFrameworkStore.getState().setTopicData(WIRE, { foo: "bar" });
    });

    expect(result.current.outOfOrderCount).toBe(0);
    expect(result.current.lagMs).toBe(0);
    expect(warnSpy).not.toHaveBeenCalled();
  });
});
