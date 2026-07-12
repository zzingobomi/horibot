// TaskProgressPanel — 디버거 wire 검증 (unit).
// TRACE(store 시딩) 가 entry 목록으로 렌더 + dot 클릭 → TOGGLE_BREAKPOINT(label)
// + breakpoints 표시 + pause/resume 게이팅 + run_to(label) + 실패 사유 표시.

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { bridge } from "@/api/bridge";
import { useFrameworkStore, type ServiceEntry } from "@/framework/store";
import { TaskProgressPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";

// task 는 backend 바인딩(GET /tasks)으로 robot 을 정함 — unit 에선 pick_and_place →
// so101 바인딩을 mock (패널의 계약 = "task 바인딩 robot 으로 wire").
vi.mock("@/hooks/useTasks", () => ({
  useTaskRobotId: () => "so101_6dof_0",
  useTasks: () => ({ tasks: [], loading: false, error: null }),
}));
const TRACE_WIRE = `stream/pick_and_place/${ROBOT_ID}/trace`;
const STATE_WIRE = `stream/pick_and_place/${ROBOT_ID}/state`;

function seed(
  status: string,
  {
    breakpoints = [],
    error = null,
    currentLabel = "",
  }: { breakpoints?: string[]; error?: string | null; currentLabel?: string } = {},
) {
  useFrameworkStore.setState({
    topicData: {
      [TRACE_WIRE]: {
        robot_id: ROBOT_ID,
        seq: 1,
        timestamp_unix: 0,
        task_name: "pick_and_place",
        entries: [
          {
            label: "detect_pick",
            kind: "detect_oriented",
            status: "completed",
            detail: "2개 후보",
            started_unix: 0,
            ended_unix: 1,
          },
          {
            label: "descend",
            kind: "move_l",
            status: "running",
            detail: "",
            started_unix: 1,
            ended_unix: null,
          },
        ],
      },
      [STATE_WIRE]: {
        robot_id: ROBOT_ID,
        seq: 1,
        timestamp_unix: 0,
        status,
        task_name: "pick_and_place",
        current_label: currentLabel,
        error,
        breakpoints,
      },
    },
    serviceData: {},
    bridgeConnected: true,
  });
}

function mockBridge() {
  return vi
    .spyOn(bridge, "callService")
    // @ts-expect-error — 테스트 stub
    .mockImplementation(async (key, _req, opts) => {
      const wk = bridge.serviceCacheKey(key, (opts as { robotId?: string })?.robotId);
      const entry: ServiceEntry = {
        success: true,
        message: "",
        data: { ok: true },
        timestamp: Date.now(),
        pending: false,
      };
      useFrameworkStore.getState().setServiceData(wk, entry);
      return entry;
    });
}

function renderPanel() {
  return render(
    <MemoryRouter initialEntries={[`/tasks/pick_and_place`]}>
      <Routes>
        <Route path="/tasks/pick_and_place" element={<TaskProgressPanel />} />
      </Routes>
    </MemoryRouter>,
  );
}

beforeEach(() => {
  useFrameworkStore.setState({ topicData: {}, serviceData: {}, bridgeConnected: true });
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("TaskProgressPanel — TRACE 디버거", () => {
  it("trace(store) → entry 목록 렌더 (label + kind + detail)", () => {
    mockBridge();
    seed("running");
    const { getAllByTestId, getByText } = renderPanel();
    expect(getAllByTestId("task-entry").length).toBe(2);
    expect(getByText("detect_pick")).toBeTruthy();
    expect(getByText("2개 후보")).toBeTruthy(); // detector 사유/결과 보임
  });

  it("dot 클릭 → TOGGLE_BREAKPOINT({label})", async () => {
    const spy = mockBridge();
    seed("running");
    const { getAllByTestId } = renderPanel();

    await act(async () => {
      fireEvent.click(getAllByTestId("task-entry-bp")[1]); // descend
    });

    const calls = spy.mock.calls.filter((c) =>
      String(c[0]).includes("toggle_breakpoint"),
    );
    expect(calls.length).toBe(1);
    expect(calls[0][1]).toEqual({ label: "descend" });
  });

  it("breakpoints(state) 표시 — 해당 label dot 에 red ring", () => {
    mockBridge();
    seed("running", { breakpoints: ["detect_pick"] });
    const { getAllByTestId } = renderPanel();
    const dots = getAllByTestId("task-entry-bp");
    expect(dots[0].className).toContain("ring-red-500");
    expect(dots[1].className).not.toContain("ring-red-500");
  });

  it("실패 = 사유가 error 박스로 표시 (침묵 금지)", () => {
    mockBridge();
    seed("failed", {
      error: "[detect_pick] 'white cube' 검출 실패 — 물체 배치 확인 후 다시 실행하세요",
    });
    const { getByTestId } = renderPanel();
    expect(getByTestId("task-error").textContent).toContain("다시 실행");
  });

  it("컨트롤 게이팅 — running 은 일시정지만, paused 는 재개/한 스텝 + run-to(label)", async () => {
    const spy = mockBridge();
    seed("running");
    const { getByTestId, queryAllByTestId, rerender } = renderPanel();
    expect((getByTestId("task-pause") as HTMLButtonElement).disabled).toBe(false);
    expect((getByTestId("task-resume") as HTMLButtonElement).disabled).toBe(true);
    expect(queryAllByTestId("task-run-to").length).toBe(0); // run-to 는 paused 만

    seed("paused", { currentLabel: "descend" });
    rerender(
      <MemoryRouter initialEntries={[`/tasks/pick_and_place`]}>
        <Routes>
          <Route path="/tasks/pick_and_place" element={<TaskProgressPanel />} />
        </Routes>
      </MemoryRouter>,
    );
    expect((getByTestId("task-pause") as HTMLButtonElement).disabled).toBe(true);
    expect((getByTestId("task-resume") as HTMLButtonElement).disabled).toBe(false);

    await act(async () => {
      fireEvent.click(queryAllByTestId("task-run-to")[1]); // descend 까지 실행
    });
    const calls = spy.mock.calls.filter((c) => String(c[0]).includes("run_to"));
    expect(calls[0][1]).toEqual({ label: "descend" });
  });
});
