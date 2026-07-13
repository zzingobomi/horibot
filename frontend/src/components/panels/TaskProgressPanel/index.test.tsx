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

// robot 바인딩 = task 계약 조회 (useTaskRobots → LIST_ROBOTS) — unit 에선 서비스
// 응답 캐시를 시딩해 so101 바인딩을 재현 (아래 LIST_ROBOTS_SEED).
const TRACE_WIRE = `stream/pick_and_place/${ROBOT_ID}/trace`;
const STATE_WIRE = `stream/pick_and_place/${ROBOT_ID}/state`;

// useTaskRobots 가 읽는 캐시 키 = 서비스 키 그대로 (robotId 없음 — 캐시 규약).
const LIST_ROBOTS_SEED: Record<string, ServiceEntry> = {
  "srv/pick_and_place/list_robots": {
    success: true,
    message: "",
    data: { robot_ids: [ROBOT_ID] },
    timestamp: 1,
    pending: false,
  },
};

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
            name: "detect_pick",
            depth: 1, // pick 안의 자식 step — 들여쓰기 렌더
            status: "completed",
            detail: "2개 후보",
            started_unix: 0,
            ended_unix: 1,
          },
          {
            name: "descend",
            depth: 1,
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
        current_name: currentLabel,
        error,
        breakpoints,
      },
    },
    serviceData: { ...LIST_ROBOTS_SEED },
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
  useFrameworkStore.setState({
    topicData: {},
    serviceData: { ...LIST_ROBOTS_SEED },
    bridgeConnected: true,
  });
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("TaskProgressPanel — TRACE 디버거", () => {
  it("trace(store) → entry 목록 렌더 (label + detail + depth 들여쓰기)", () => {
    mockBridge();
    seed("running");
    const { getAllByTestId, getByText } = renderPanel();
    const entries = getAllByTestId("task-entry");
    expect(entries.length).toBe(2);
    expect(getByText("detect_pick")).toBeTruthy();
    expect(getByText("2개 후보")).toBeTruthy(); // detector 사유/결과 보임
    // depth=1 → 들여쓰기 (중첩 step 시각 구분)
    expect((entries[0] as HTMLElement).style.marginLeft).toBe("14px");
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
    expect(calls[0][1]).toEqual({ name: "descend" });
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
    expect(calls[0][1]).toEqual({ name: "descend" });
  });

  it("전체 단계 미리보기 → PREVIEW 호출 + 예상 단계 목록 렌더 (live trace 없을 때)", async () => {
    vi.spyOn(bridge, "callService")
      // @ts-expect-error — 테스트 stub
      .mockImplementation(async (key, _req, opts) => {
        const wk = bridge.serviceCacheKey(
          key,
          (opts as { robotId?: string })?.robotId,
        );
        const isPreview = String(key).includes("/preview");
        const entry: ServiceEntry = {
          success: true,
          message: "",
          data: isPreview
            ? {
                steps: [
                  { name: "plan_pick", title: "집기 계획", depth: 0, status: "running", started_unix: 0 },
                  { name: "detect", title: "검출", depth: 1, status: "running", started_unix: 0 },
                ],
              }
            : { ok: true },
          timestamp: Date.now(),
          pending: false,
        };
        useFrameworkStore.getState().setServiceData(wk, entry);
        return entry;
      });
    // idle (live trace 없음) — LIST_ROBOTS 만 시딩
    useFrameworkStore.setState({
      topicData: {},
      serviceData: { ...LIST_ROBOTS_SEED },
      bridgeConnected: true,
    });
    const { getByTestId, getAllByTestId, getByText } = renderPanel();

    await act(async () => {
      fireEvent.click(getByTestId("task-preview"));
    });

    const items = getAllByTestId("task-preview-entry");
    expect(items.length).toBe(2);
    expect(getByText("집기 계획")).toBeTruthy();
    // depth=1 은 들여쓰기
    expect((items[1] as HTMLElement).style.marginLeft).toBe("14px");
  });
});
