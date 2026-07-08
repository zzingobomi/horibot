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

let mockRobots: { id: string; type: string }[] = [];
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
