// AutoHideHeader — `+ 패널 추가` 의 capability disabled (설계 point 4, route 조건부).
// visible 은 CSS transform 만 바꾸고 버튼은 항상 DOM 에 있으므로, mousemove reveal
// 없이 드롭다운을 열어 disabled 판정을 검증할 수 있음 (비-brittle).
//
// 핵심 계약:
// - ambient robot 이 요구 capability 를 못 가지면 그 항목 disabled + 사유.
// - ambientCapabilities=null(=/tasks·/world, 대상 robot 없음) → 아무것도 disable 안 함.
// - 요구 없는 패널 / capability 충족 robot → 정상(클릭 가능).

import { describe, it, expect, vi } from "vitest";
import { render, fireEvent } from "@testing-library/react";
import type { DockviewApi } from "dockview";
import { AutoHideHeader } from "./AutoHideHeader";
import type { PanelSpec } from "./ModeDockview";

function fakeApi(): DockviewApi {
  return {
    panels: [],
    onDidAddPanel: () => ({ dispose() {} }),
    onDidRemovePanel: () => ({ dispose() {} }),
  } as unknown as DockviewApi;
}

const MOTION: PanelSpec = {
  id: "add-motion",
  component: "motion",
  title: "Motion",
  width: 1,
  height: 1,
};
const SCAN: PanelSpec = {
  id: "add-scan",
  component: "scan",
  title: "Scan",
  width: 1,
  height: 1,
  requiredCapabilities: ["rgbd"],
};

function renderHeader(ambientCapabilities: string[] | null) {
  const onAddPanel = vi.fn();
  const utils = render(
    <AutoHideHeader
      api={fakeApi()}
      candidates={[MOTION, SCAN]}
      ambientCapabilities={ambientCapabilities}
      onAddPanel={onAddPanel}
      onResetLayout={vi.fn()}
    />,
  );
  // 드롭다운 열기 (버튼은 visible 무관하게 DOM 에 있음)
  fireEvent.click(utils.getByRole("button", { name: /패널 추가/ }));
  return { ...utils, onAddPanel };
}

describe("AutoHideHeader — capability disabled", () => {
  it("ambient robot 이 rgbd 없음 → Scan disabled + 사유, 클릭 무시. 요구 없는 Motion 은 활성", () => {
    const { getByTestId, getByText, onAddPanel } = renderHeader(["move"]);

    const scanBtn = getByTestId("add-panel-disabled");
    expect(scanBtn).toBeDisabled();
    expect(scanBtn.textContent).toContain("Scan");
    expect(getByText("RGB-D 카메라 필요")).toBeTruthy();

    // 요구 없는 패널은 활성
    const motionBtn = getByTestId("add-panel-item");
    expect(motionBtn).not.toBeDisabled();

    // disabled 클릭해도 add 안 됨
    fireEvent.click(scanBtn);
    expect(onAddPanel).not.toHaveBeenCalled();
  });

  it("ambientCapabilities=null (/tasks·/world) → 대상 robot 없음, 아무것도 disable 안 함", () => {
    const { queryByTestId, getAllByTestId } = renderHeader(null);
    expect(queryByTestId("add-panel-disabled")).toBeNull();
    // Motion + Scan 둘 다 활성 항목
    expect(getAllByTestId("add-panel-item")).toHaveLength(2);
  });

  it("ambient robot 이 rgbd 있음 → Scan 활성, 클릭 시 onAddPanel 호출", () => {
    const { queryByTestId, getByText, onAddPanel } = renderHeader(["move", "rgbd"]);
    expect(queryByTestId("add-panel-disabled")).toBeNull();

    fireEvent.click(getByText("Scan"));
    expect(onAddPanel).toHaveBeenCalledWith(SCAN);
  });
});
