// frontend_v2.md §12.2 useTopic — 1 invariant.

import { beforeEach, describe, expect, it } from "vitest";
import { act, renderHook } from "@testing-library/react";
import { useFrameworkStore } from "./store";
import { useTopic } from "./topic";

describe("useTopic", () => {
  beforeEach(() => {
    useFrameworkStore.setState({
      topicData: {},
      serviceData: {},
      bridgeConnected: false,
    });
  });

  // spec frontend_v2.md §12.2 — invariant: store.setTopicData → useTopic reactive read
  it("store.setTopicData 후 useTopic reactive 갱신", () => {
    const wire = "stream/motion/so101_6dof_0/tcp_state";
    const { result } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useTopic(wire as any, "so101_6dof_0"),
    );

    expect(result.current).toBeNull();

    act(() => {
      useFrameworkStore.getState().setTopicData(wire, {
        robot_id: "so101_6dof_0",
        seq: 1,
        position: [0, 0, 0.3],
      });
    });

    expect(result.current).toMatchObject({
      robot_id: "so101_6dof_0",
      seq: 1,
    });
  });
});
