// HandoverPanel — 실행 컨트롤 wire 검증 (unit).
// [실행] → HANDOVER_RUN 에 typed RunRequest (stop_before_receive 포함 — 기본 ON
// = 실물 검증 프로토콜: omx 집기+제시만 먼저) / [중지] → HANDOVER_STOP. 거부/
// 실패 사유 표시 (침묵 금지).

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render } from "@testing-library/react";
import { bridge } from "@/api/bridge";
import { useFrameworkStore, type ServiceEntry } from "@/framework/store";
import { HandoverPanel } from "./index";

// robot 바인딩 = task 계약 조회 (useTaskRobots → LIST_ROBOTS) — unit 에선 서비스
// 응답 캐시 시딩으로 (so101, omx) 바인딩 재현.
const LIST_ROBOTS_SEED: Record<string, ServiceEntry> = {
  "srv/handover/list_robots": {
    success: true,
    message: "",
    data: { robot_ids: ["so101_6dof_0", "omx_f_0"] },
    timestamp: 1,
    pending: false,
  },
};

function mockBridge(dataByKey: Record<string, unknown> = {}) {
  return vi
    .spyOn(bridge, "callService")
    // @ts-expect-error — 테스트 stub
    .mockImplementation(async (key, _req, opts) => {
      const wk = bridge.serviceCacheKey(key, (opts as { robotId?: string })?.robotId);
      const entry: ServiceEntry = {
        success: true,
        message: "",
        data: dataByKey[String(key)] ?? { ok: true, accepted: true },
        timestamp: Date.now(),
        pending: false,
      };
      useFrameworkStore.getState().setServiceData(wk, entry);
      return entry;
    });
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

describe("HandoverPanel", () => {
  it("실행 → HANDOVER_RUN 에 typed RunRequest — stop_before_receive 기본 ON", async () => {
    const spy = mockBridge({
      "srv/handover/run": { accepted: true, message: "" },
    });
    const { getByTestId } = render(<HandoverPanel />);

    // 기본값: pick="blue block" (봉 프롬프트), stop_before_receive=true
    expect(
      (getByTestId("handover-stop-before-receive") as HTMLInputElement).checked,
    ).toBe(true);
    await act(async () => {
      fireEvent.click(getByTestId("handover-run"));
    });

    const calls = spy.mock.calls.filter(
      (c) => String(c[0]) === "srv/handover/run",
    );
    expect(calls.length).toBe(1);
    expect(calls[0][1]).toEqual({
      pick_object: "blue block",
      place_object: "",
      stop_before_receive: true, // 첫 런 안전 기본 — omx 집기+제시까지만
    });
  });

  it("stop_before_receive 해제 → false 로 전달 (풀 시퀀스)", async () => {
    const spy = mockBridge({
      "srv/handover/run": { accepted: true, message: "" },
    });
    const { getByTestId } = render(<HandoverPanel />);

    fireEvent.click(getByTestId("handover-stop-before-receive"));
    fireEvent.change(getByTestId("handover-place"), {
      target: { value: "white box" },
    });
    await act(async () => {
      fireEvent.click(getByTestId("handover-run"));
    });

    const calls = spy.mock.calls.filter(
      (c) => String(c[0]) === "srv/handover/run",
    );
    expect(calls[0][1]).toEqual({
      pick_object: "blue block",
      place_object: "white box",
      stop_before_receive: false,
    });
  });

  it("실행 거부 → 사유 표시 (침묵 금지)", async () => {
    mockBridge({
      "srv/handover/run": {
        accepted: false,
        message: "이미 실행 중 (handover)",
      },
    });
    const { getByTestId } = render(<HandoverPanel />);
    await act(async () => {
      fireEvent.click(getByTestId("handover-run"));
    });
    expect(getByTestId("handover-msg").textContent).toContain("이미 실행 중");
  });

  it("중지 → HANDOVER_STOP, 실패 시 사유 표시", async () => {
    const spy = mockBridge({
      "srv/handover/stop": { ok: false, message: "실행 중인 run 없음" },
    });
    const { getByTestId } = render(<HandoverPanel />);
    await act(async () => {
      fireEvent.click(getByTestId("handover-stop"));
    });
    const calls = spy.mock.calls.filter(
      (c) => String(c[0]) === "srv/handover/stop",
    );
    expect(calls.length).toBe(1);
    expect(getByTestId("handover-msg").textContent).toContain("실행 중인 run 없음");
  });

  it("참여 robot 명부 표시 (giver → receiver 순)", () => {
    mockBridge();
    const { getByTestId } = render(<HandoverPanel />);
    // TASK_ROBOTS = (receiver, giver) — 표시는 giver → receiver 로 뒤집는다
    expect(getByTestId("handover-robots").textContent).toBe(
      "omx_f_0 → so101_6dof_0",
    );
  });

  it("pick 비면 실행 버튼 disabled (필수 param)", () => {
    mockBridge();
    const { getByTestId } = render(<HandoverPanel />);
    fireEvent.change(getByTestId("handover-pick"), { target: { value: " " } });
    expect((getByTestId("handover-run") as HTMLButtonElement).disabled).toBe(true);
  });
});
