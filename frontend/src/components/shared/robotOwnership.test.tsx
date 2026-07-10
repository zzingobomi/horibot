// robot 소유권 모델([[robot_ownership_model]]) 불변식 검증.
// - 바인딩은 오직 params.robotId (환경 아님) → 미바인딩이면 Select Robot, 바인딩이면
//   패널 렌더 + useRobotId 가 그 id 반환.
// - 셀렉터 변경 = 패널이 자기 바인딩을 기록(updateParameters). robot 1개면 셀렉터 숨김.

import { describe, it, expect, vi, beforeEach } from "vitest";
import { render, fireEvent } from "@testing-library/react";
import type {
  IDockviewPanelProps,
  IDockviewPanelHeaderProps,
} from "dockview";
import { withRobotOwnership, RobotTab } from "./robotOwnership";
import { useRobotId } from "@/hooks/useRobotId";
import { usePanelInstanceStore } from "@/stores/panelInstanceStore";

let mockRobots: { id: string; type: string; capabilities?: string[] }[] = [];
vi.mock("@/hooks/useRobots", () => ({
  useRobots: () => ({ robots: mockRobots, loading: false, error: null }),
}));

function Probe() {
  const id = useRobotId();
  return <div data-testid="probe">{id}</div>;
}

function contentProps(params: Record<string, unknown>): IDockviewPanelProps {
  return {
    params,
    api: { updateParameters: vi.fn() },
    containerApi: {},
  } as unknown as IDockviewPanelProps;
}

function tabProps(params: Record<string, unknown>): IDockviewPanelHeaderProps {
  return {
    params,
    api: {
      title: "Robot State",
      onDidTitleChange: () => ({ dispose() {} }),
      updateParameters: vi.fn(),
    },
    containerApi: {},
    tabLocation: "header",
  } as unknown as IDockviewPanelHeaderProps;
}

beforeEach(() => {
  mockRobots = [
    { id: "omx_f_0", type: "omx_f" },
    { id: "so101_6dof_0", type: "so101_6dof" },
  ];
});

describe("withRobotOwnership", () => {
  it("robotId 미바인딩 → Select Robot 빈 상태, 패널 내용 렌더 안 함", () => {
    const Wrapped = withRobotOwnership(Probe);
    const { queryByTestId, getByText } = render(<Wrapped {...contentProps({})} />);
    expect(queryByTestId("probe")).toBeNull();
    expect(getByText(/대상 robot/)).toBeTruthy();
  });

  it("robotId 바인딩 → 패널 렌더 + useRobotId 가 그 id 반환", () => {
    const Wrapped = withRobotOwnership(Probe);
    const { getByTestId } = render(
      <Wrapped {...contentProps({ robotId: "omx_f_0" })} />,
    );
    expect(getByTestId("probe").textContent).toBe("omx_f_0");
  });

  it("빈 상태에서 robot 선택 → api.updateParameters(robotId) 로 자기 바인딩 기록", () => {
    const Wrapped = withRobotOwnership(Probe);
    const props = contentProps({});
    const { getByRole } = render(<Wrapped {...props} />);
    fireEvent.change(getByRole("combobox"), {
      target: { value: "so101_6dof_0" },
    });
    expect(props.api.updateParameters).toHaveBeenCalledWith({
      robotId: "so101_6dof_0",
    });
  });
});

// capability gating — 항상-정확한 1차 방어 (route/페이지 무관). ambient robot 이
// 없는 /tasks 에서도 패널이 실제 바인딩한 robot 을 검사해 잡는다.
describe("withRobotOwnership — capability gating", () => {
  beforeEach(() => {
    mockRobots = [
      { id: "omx_f_0", type: "omx_f", capabilities: ["move", "calibrate"] },
      {
        id: "so101_6dof_0",
        type: "so101_6dof",
        capabilities: ["move", "calibrate", "rgbd"],
      },
    ];
  });

  it("바인딩 robot 이 요구 capability 없음 → unsupported empty (패널 렌더 안 함, 사유 표시)", () => {
    const Wrapped = withRobotOwnership(Probe, { requiredCapabilities: ["rgbd"] });
    const { queryByTestId, getByTestId, getByText } = render(
      <Wrapped {...contentProps({ robotId: "omx_f_0" })} />,
    );
    expect(queryByTestId("probe")).toBeNull();
    expect(getByTestId("capability-unsupported")).toBeTruthy();
    expect(getByText(/RGB-D 카메라 필요/)).toBeTruthy();
  });

  it("바인딩 robot 이 요구 capability 있음 → 패널 정상 렌더", () => {
    const Wrapped = withRobotOwnership(Probe, { requiredCapabilities: ["rgbd"] });
    const { getByTestId, queryByTestId } = render(
      <Wrapped {...contentProps({ robotId: "so101_6dof_0" })} />,
    );
    expect(getByTestId("probe").textContent).toBe("so101_6dof_0");
    expect(queryByTestId("capability-unsupported")).toBeNull();
  });

  it("override 사유가 있으면 파생 문구 대신 그것을 표시", () => {
    const Wrapped = withRobotOwnership(Probe, {
      requiredCapabilities: ["rgbd"],
      unavailableReason: "커스텀 안내",
    });
    const { getByText } = render(
      <Wrapped {...contentProps({ robotId: "omx_f_0" })} />,
    );
    expect(getByText("커스텀 안내")).toBeTruthy();
  });

  it("robot 목록 미상(로딩 전) → capability 판정 보류, 패널 렌더 (false 미지원 flash 방지)", () => {
    mockRobots = [];
    const Wrapped = withRobotOwnership(Probe, { requiredCapabilities: ["rgbd"] });
    const { getByTestId } = render(
      <Wrapped {...contentProps({ robotId: "omx_f_0" })} />,
    );
    expect(getByTestId("probe").textContent).toBe("omx_f_0");
  });
});

// scenePart 배선 — HOC 가 chokepoint 로 panelInstanceStore 에 인스턴스 등록.
// ([docs/scene_contribution_architecture.md] — Canvas 의 ScenePartHost 가 소비)
describe("withRobotOwnership — 인스턴스 등록 (scenePart)", () => {
  beforeEach(() => {
    usePanelInstanceStore.setState({ instances: {} });
  });

  const instanceList = () => Object.values(usePanelInstanceStore.getState().instances);

  it("바인딩 렌더 시 등록 (panelKind + robotId), unmount 시 해제", () => {
    const Wrapped = withRobotOwnership(Probe, { panelKind: "livePointCloud" });
    const { unmount } = render(<Wrapped {...contentProps({ robotId: "omx_f_0" })} />);
    expect(instanceList()).toEqual([
      { panelKind: "livePointCloud", robotId: "omx_f_0" },
    ]);
    unmount();
    expect(instanceList()).toEqual([]);
  });

  it("robotId 미바인딩(Select Robot) → 등록 안 함", () => {
    const Wrapped = withRobotOwnership(Probe, { panelKind: "livePointCloud" });
    render(<Wrapped {...contentProps({})} />);
    expect(instanceList()).toEqual([]);
  });

  it("capability unsupported → 등록 안 함 (씬 조각도 자동 미표시)", () => {
    mockRobots = [{ id: "omx_f_0", type: "omx_f", capabilities: ["move"] }];
    const Wrapped = withRobotOwnership(Probe, {
      panelKind: "livePointCloud",
      requiredCapabilities: ["rgbd"],
    });
    render(<Wrapped {...contentProps({ robotId: "omx_f_0" })} />);
    expect(instanceList()).toEqual([]);
  });

  it("robot 스위칭(rerender 새 params) → 등록 robotId 갱신", () => {
    const Wrapped = withRobotOwnership(Probe, { panelKind: "livePointCloud" });
    const { rerender } = render(
      <Wrapped {...contentProps({ robotId: "omx_f_0" })} />,
    );
    rerender(<Wrapped {...contentProps({ robotId: "so101_6dof_0" })} />);
    expect(instanceList()).toEqual([
      { panelKind: "livePointCloud", robotId: "so101_6dof_0" },
    ]);
  });

  it("panelKind 미지정(옛 호출 형태) → 등록 없이 정상 렌더 (하위 호환)", () => {
    const Wrapped = withRobotOwnership(Probe);
    const { getByTestId } = render(
      <Wrapped {...contentProps({ robotId: "omx_f_0" })} />,
    );
    expect(getByTestId("probe")).toBeTruthy();
    expect(instanceList()).toEqual([]);
  });
});

describe("RobotTab", () => {
  it("robot 2개 이상 → 셀렉터 노출, 변경 시 updateParameters", () => {
    const props = tabProps({ robotId: "omx_f_0" });
    const { getByRole } = render(<RobotTab {...props} />);
    fireEvent.change(getByRole("combobox"), {
      target: { value: "so101_6dof_0" },
    });
    expect(props.api.updateParameters).toHaveBeenCalledWith({
      robotId: "so101_6dof_0",
    });
  });

  it("robot 1개 → 셀렉터 숨김", () => {
    mockRobots = [{ id: "omx_f_0", type: "omx_f" }];
    const { queryByRole } = render(<RobotTab {...tabProps({ robotId: "omx_f_0" })} />);
    expect(queryByRole("combobox")).toBeNull();
  });
});
