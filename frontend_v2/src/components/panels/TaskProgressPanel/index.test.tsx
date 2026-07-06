// TaskProgressPanel — 디버거 wire 검증 (unit).
// preview 로 publish 된 tree(store 시딩) 가 step 목록으로 렌더 + dot 클릭 →
// TOGGLE_BREAKPOINT + breakpoints 표시 + pause/resume 게이팅.

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { bridge } from "@/api/bridge";
import { useFrameworkStore, type ServiceEntry } from "@/framework/store";
import { TaskProgressPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";
const TREE_WIRE = `stream/task/${ROBOT_ID}/tree`;
const STATE_WIRE = `stream/task/${ROBOT_ID}/state`;

function seed(status: string, breakpoints: string[] = []) {
  useFrameworkStore.setState({
    topicData: {
      [TREE_WIRE]: {
        robot_id: ROBOT_ID,
        seq: 1,
        timestamp_unix: 0,
        task_name: "pick_and_place",
        steps: [
          { id: "s1", label: "detect", type: "GroundedDetect" },
          { id: "s2", label: "grasp", type: "GraspPolicy" },
        ],
      },
      [STATE_WIRE]: {
        robot_id: ROBOT_ID,
        seq: 1,
        timestamp_unix: 0,
        status,
        task_name: "pick_and_place",
        current_step_id: "",
        step_statuses: {},
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
      const wk = bridge.expand(key, (opts as { robotId?: string })?.robotId ?? ROBOT_ID);
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
    <MemoryRouter initialEntries={[`/robots/${ROBOT_ID}`]}>
      <Routes>
        <Route path="/robots/:id" element={<TaskProgressPanel />} />
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

describe("TaskProgressPanel — 디버거", () => {
  it("preview tree(store) → step 목록 렌더 (실행 전에 목록이 뜬다)", () => {
    mockBridge();
    seed("idle");
    const { getAllByTestId } = renderPanel();
    expect(getAllByTestId("task-step").length).toBe(2);
  });

  it("dot 클릭 → TOGGLE_BREAKPOINT(robot_id, step_id)", async () => {
    const spy = mockBridge();
    seed("idle");
    const { getAllByTestId } = renderPanel();

    await act(async () => {
      fireEvent.click(getAllByTestId("task-step-bp")[1]); // s2
    });

    const calls = spy.mock.calls.filter((c) =>
      String(c[0]).includes("toggle_breakpoint"),
    );
    expect(calls.length).toBe(1);
    expect(calls[0][1]).toEqual({ robot_id: ROBOT_ID, step_id: "s2" });
  });

  it("breakpoints(state) 표시 — 해당 step dot 에 red ring", () => {
    mockBridge();
    seed("idle", ["s1"]);
    const { getAllByTestId } = renderPanel();
    const dots = getAllByTestId("task-step-bp");
    expect(dots[0].className).toContain("ring-red-500");
    expect(dots[1].className).not.toContain("ring-red-500");
  });

  it("컨트롤 게이팅 — running 은 일시정지만, paused 는 재개/한 스텝 + run-to", async () => {
    const spy = mockBridge();
    seed("running");
    const { getByTestId, queryAllByTestId, rerender } = renderPanel();
    expect((getByTestId("task-pause") as HTMLButtonElement).disabled).toBe(false);
    expect((getByTestId("task-resume") as HTMLButtonElement).disabled).toBe(true);
    expect(queryAllByTestId("task-run-to").length).toBe(0); // run-to 는 paused 만

    seed("paused");
    rerender(
      <MemoryRouter initialEntries={[`/robots/${ROBOT_ID}`]}>
        <Routes>
          <Route path="/robots/:id" element={<TaskProgressPanel />} />
        </Routes>
      </MemoryRouter>,
    );
    expect((getByTestId("task-pause") as HTMLButtonElement).disabled).toBe(true);
    expect((getByTestId("task-resume") as HTMLButtonElement).disabled).toBe(false);

    await act(async () => {
      fireEvent.click(queryAllByTestId("task-run-to")[1]); // s2 까지 실행
    });
    const calls = spy.mock.calls.filter((c) => String(c[0]).includes("run_to"));
    expect(calls[0][1]).toEqual({ robot_id: ROBOT_ID, step_id: "s2" });
  });
});
