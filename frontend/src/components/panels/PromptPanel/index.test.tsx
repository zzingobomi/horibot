// PromptPanel — 자연어 → LLM parse → PnP run wire 검증 (unit).
// 클릭 → 올바른 ServiceKey + payload 로 callService 되는지 (WaypointPanel 철학 동일).
// 실 파싱 정확도/실행은 실 backend + hardware (§17.5).

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render, waitFor } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { bridge } from "@/api/bridge";
import { useFrameworkStore, type ServiceEntry } from "@/framework/store";
import { PromptPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";

// task 는 backend 바인딩(GET /tasks)으로 robot 을 정함 — unit 에선 pick_and_place →
// so101 바인딩을 mock (패널의 계약 = "task 바인딩 robot 으로 wire/명령").
vi.mock("@/hooks/useTasks", () => ({
  useTaskRobotId: () => "so101_6dof_0",
  useTasks: () => ({ tasks: [], loading: false, error: null }),
}));

function respond(key: string): unknown {
  if (key.includes("parse_command")) {
    return {
      ok: true,
      parsed: { pick_object: "white cube", place_object: "blue box" },
      message: "",
    };
  }
  if (key.includes("/run")) return { accepted: true, message: "" };
  if (key.includes("/preview")) return { ok: true, message: "" };
  return { ok: true };
}

function mockBridge() {
  return (
    vi
      .spyOn(bridge, "callService")
      // @ts-expect-error — 테스트 stub, 응답 shape 는 respond() 가 책임
      .mockImplementation(async (key, _req, opts) => {
        const wk = bridge.serviceCacheKey(
          key,
          (opts as { robotId?: string })?.robotId,
        );
        const entry: ServiceEntry = {
          success: true,
          message: "",
          data: respond(String(key)),
          timestamp: Date.now(),
          pending: false,
        };
        useFrameworkStore.getState().setServiceData(wk, entry);
        return entry;
      })
  );
}

function renderPanel() {
  return render(
    <MemoryRouter initialEntries={[`/robots/${ROBOT_ID}`]}>
      <Routes>
        <Route path="/robots/:id" element={<PromptPanel />} />
      </Routes>
    </MemoryRouter>,
  );
}

beforeEach(() => {
  useFrameworkStore.setState({
    topicData: {},
    serviceData: {},
    bridgeConnected: true,
  });
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("PromptPanel", () => {
  it("패널 렌더", () => {
    mockBridge();
    const { getByTestId } = renderPanel();
    expect(getByTestId("prompt-panel")).toBeTruthy();
    expect(getByTestId("prompt-input")).toBeTruthy();
  });

  it("파싱 → LLM_PARSE_COMMAND 를 입력 text 로 call + 결과 표시", async () => {
    const spy = mockBridge();
    const { getByTestId } = renderPanel();

    act(() => {
      fireEvent.change(getByTestId("prompt-input"), {
        target: { value: "흰색 작고 네모난 큐브를 파란 상자에 둬" },
      });
    });
    await act(async () => {
      fireEvent.click(getByTestId("prompt-parse"));
    });

    await waitFor(() => {
      const calls = spy.mock.calls.filter((c) =>
        String(c[0]).includes("parse_command"),
      );
      expect(calls.length).toBeGreaterThan(0);
      const req = calls[0][1] as { text: string };
      expect(req.text).toBe("흰색 작고 네모난 큐브를 파란 상자에 둬");
    });
    // 파싱 결과 (pick/place) 표시
    await waitFor(() => expect(getByTestId("prompt-parsed")).toBeTruthy());

    // v1 디버거 플로우 — 파싱 성공 시 자동 TASK_PREVIEW (tree publish, 실행 X).
    // 이게 빠지면 step 목록이 실행 전에 안 떠서 브레이크포인트를 미리 못 박음.
    await waitFor(() => {
      const previews = spy.mock.calls.filter((c) =>
        String(c[0]).endsWith("/preview"),
      );
      expect(previews.length).toBe(1);
      const req = previews[0][1] as {
        robot_id: string;
        task_name: string;
        params: Record<string, string>;
      };
      expect(req.robot_id).toBe(ROBOT_ID);
      expect(req.task_name).toBe("pick_and_place");
      expect(req.params.pick_object).toBe("white cube");
      // run 은 아직 호출 안 됨 (preview ≠ 실행)
      expect(
        spy.mock.calls.filter((c) => String(c[0]).endsWith("/run")).length,
      ).toBe(0);
    });
  });

  it("실행 → TASK_RUN 을 pick_and_place + parsed params 로 call", async () => {
    const spy = mockBridge();
    const { getByTestId } = renderPanel();

    act(() => {
      fireEvent.change(getByTestId("prompt-input"), {
        target: { value: "흰색 작고 네모난 큐브를 파란 상자에 둬" },
      });
    });
    await act(async () => {
      fireEvent.click(getByTestId("prompt-parse"));
    });
    await waitFor(() => expect(getByTestId("prompt-parsed")).toBeTruthy());

    await act(async () => {
      fireEvent.click(getByTestId("prompt-run"));
    });

    await waitFor(() => {
      const calls = spy.mock.calls.filter((c) => String(c[0]).endsWith("/run"));
      expect(calls.length).toBeGreaterThan(0);
      const req = calls[0][1] as {
        robot_id: string;
        task_name: string;
        params: Record<string, string>;
      };
      expect(req.robot_id).toBe(ROBOT_ID);
      expect(req.task_name).toBe("pick_and_place");
      expect(req.params.pick_object).toBe("white cube");
      expect(req.params.place_object).toBe("blue box");
    });
  });
});
