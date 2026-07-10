// ScenePartHost — 인스턴스 목록(데이터) × registry scene 선언 → 인스턴스별 마운트.
// scenePart 안이 패널과 같은 멘탈모델(useRobotId)인지가 핵심 계약.

import { describe, it, expect, vi, beforeEach } from "vitest";
import { render, act } from "@testing-library/react";
import { usePanelInstanceStore } from "@/stores/panelInstanceStore";
import { useRobotId } from "@/hooks/useRobotId";
import { ScenePartHost } from "./ScenePartHost";

function ProbeScene() {
  const robotId = useRobotId();
  return <div data-testid="scene-part">{robotId}</div>;
}

// registry mock — scenePart 선언이 있는 kind(livePointCloud)와 없는 kind(motion)
vi.mock("@/components/panels/registry", () => ({
  PANEL_CATALOG: {
    livePointCloud: { title: "Live PointCloud", width: 1, height: 1, scenePart: ProbeScene },
    motion: { title: "Motion", width: 1, height: 1 },
  },
}));

beforeEach(() => {
  usePanelInstanceStore.setState({ instances: {} });
});

describe("ScenePartHost", () => {
  it("scene 선언된 kind 의 인스턴스 → RobotProvider 로 감싸 마운트 (useRobotId 동작)", () => {
    usePanelInstanceStore.getState().register("i1", {
      panelKind: "livePointCloud",
      robotId: "so101_6dof_0",
    });
    const { getByTestId } = render(<ScenePartHost />);
    expect(getByTestId("scene-part").textContent).toBe("so101_6dof_0");
  });

  it("scene 미선언 kind 의 인스턴스 → 아무것도 마운트 안 함", () => {
    usePanelInstanceStore.getState().register("i1", {
      panelKind: "motion",
      robotId: "a",
    });
    const { queryByTestId } = render(<ScenePartHost />);
    expect(queryByTestId("scene-part")).toBeNull();
  });

  it("같은 패널 2 인스턴스(robot A/B) → 조각 2개, 각자 자기 robot", () => {
    const s = usePanelInstanceStore.getState();
    s.register("i1", { panelKind: "livePointCloud", robotId: "a" });
    s.register("i2", { panelKind: "livePointCloud", robotId: "b" });
    const { getAllByTestId } = render(<ScenePartHost />);
    const texts = getAllByTestId("scene-part").map((el) => el.textContent);
    expect(texts.sort()).toEqual(["a", "b"]);
  });

  it("인스턴스 해제(패널 닫힘) → 조각도 사라짐 (reactive)", () => {
    usePanelInstanceStore.getState().register("i1", {
      panelKind: "livePointCloud",
      robotId: "a",
    });
    const { queryByTestId } = render(<ScenePartHost />);
    expect(queryByTestId("scene-part")).toBeTruthy();
    act(() => usePanelInstanceStore.getState().unregister("i1"));
    expect(queryByTestId("scene-part")).toBeNull();
  });
});
