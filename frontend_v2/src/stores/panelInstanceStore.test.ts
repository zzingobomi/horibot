import { describe, it, expect, beforeEach } from "vitest";
import { usePanelInstanceStore } from "./panelInstanceStore";

beforeEach(() => {
  usePanelInstanceStore.setState({ instances: {} });
});

describe("panelInstanceStore", () => {
  it("register → instances 에 반영, unregister → 제거", () => {
    const s = usePanelInstanceStore.getState();
    s.register("k1", { panelKind: "livePointCloud", robotId: "so101_6dof_0" });
    expect(usePanelInstanceStore.getState().instances["k1"]).toEqual({
      panelKind: "livePointCloud",
      robotId: "so101_6dof_0",
    });
    s.unregister("k1");
    expect(usePanelInstanceStore.getState().instances["k1"]).toBeUndefined();
  });

  it("같은 패널 2 인스턴스(robot A/B) 공존 — 인스턴스 의미론", () => {
    const s = usePanelInstanceStore.getState();
    s.register("k1", { panelKind: "motion", robotId: "a" });
    s.register("k2", { panelKind: "motion", robotId: "b" });
    expect(Object.keys(usePanelInstanceStore.getState().instances)).toHaveLength(2);
  });

  it("동일 값 재등록은 no-op (state ref 불변 — Canvas 재렌더 방지)", () => {
    const s = usePanelInstanceStore.getState();
    s.register("k1", { panelKind: "motion", robotId: "a" });
    const before = usePanelInstanceStore.getState().instances;
    s.register("k1", { panelKind: "motion", robotId: "a" });
    expect(usePanelInstanceStore.getState().instances).toBe(before);
  });

  it("robotId 변경 재등록은 갱신", () => {
    const s = usePanelInstanceStore.getState();
    s.register("k1", { panelKind: "motion", robotId: "a" });
    s.register("k1", { panelKind: "motion", robotId: "b" });
    expect(usePanelInstanceStore.getState().instances["k1"].robotId).toBe("b");
  });

  it("없는 키 unregister 는 no-op (state ref 불변)", () => {
    const before = usePanelInstanceStore.getState().instances;
    usePanelInstanceStore.getState().unregister("ghost");
    expect(usePanelInstanceStore.getState().instances).toBe(before);
  });
});
