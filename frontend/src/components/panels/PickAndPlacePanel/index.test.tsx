// PickAndPlacePanel — 실행 컨트롤 wire 검증 (unit).
// [파싱] → LLM_PARSE_COMMAND 가 폼을 채움 (실행 아님) / [실행] → PICKANDPLACE_RUN
// 에 typed RunRequest / [중지] → PICKANDPLACE_STOP. 거부/실패 사유 표시.

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render } from "@testing-library/react";
import { bridge } from "@/api/bridge";
import { useFrameworkStore, type ServiceEntry } from "@/framework/store";
import { PickAndPlacePanel } from "./index";

// robot 바인딩 = task 계약 조회 (useTaskRobots → LIST_ROBOTS) — unit 에선 서비스
// 응답 캐시 시딩으로 so101 바인딩 재현.
const LIST_ROBOTS_SEED: Record<string, ServiceEntry> = {
  "srv/pick_and_place/list_robots": {
    success: true,
    message: "",
    data: { robot_ids: ["so101_6dof_0"] },
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

describe("PickAndPlacePanel", () => {
  it("파싱 → LLM_PARSE_COMMAND 호출 + 폼 채움 (RUN 은 0회)", async () => {
    const spy = mockBridge({
      "srv/llm/parse_command": {
        ok: true,
        parsed: { pick_object: "white cube", place_object: "blue box" },
      },
    });
    const { getByTestId } = render(<PickAndPlacePanel />);

    await act(async () => {
      fireEvent.click(getByTestId("pnp-parse"));
    });

    const parseCalls = spy.mock.calls.filter((c) =>
      String(c[0]).includes("parse_command"),
    );
    expect(parseCalls.length).toBe(1);
    expect((getByTestId("pnp-pick") as HTMLInputElement).value).toBe("white cube");
    expect((getByTestId("pnp-place") as HTMLInputElement).value).toBe("blue box");
    const runCalls = spy.mock.calls.filter((c) => String(c[0]).includes("/run"));
    expect(runCalls.length).toBe(0); // 파싱은 실행이 아님 — 사용자 확인 후 [실행]
  });

  it("실행 → PICKANDPLACE_RUN 에 typed RunRequest (params dict 아님)", async () => {
    const spy = mockBridge({
      "srv/pick_and_place/run": { accepted: true, message: "" },
    });
    const { getByTestId } = render(<PickAndPlacePanel />);

    fireEvent.change(getByTestId("pnp-pick"), { target: { value: "white cube" } });
    fireEvent.change(getByTestId("pnp-place"), { target: { value: "" } });
    await act(async () => {
      fireEvent.click(getByTestId("pnp-run"));
    });

    const calls = spy.mock.calls.filter((c) =>
      String(c[0]) === "srv/pick_and_place/run",
    );
    expect(calls.length).toBe(1);
    expect(calls[0][1]).toEqual({
      pick_object: "white cube",
      place_object: "",
      build_world: false, // 기본 off — "빨리 픽앤플레이스만" 이 기본 (2026-07-18)
      world_voxel_size: 0.002, // 기본 2mm (계측 전 기본 묵시 변경 안 함)
    });
  });

  it("월드 갱신 체크 → build_world=true 로 실행 + localStorage 기억", async () => {
    const spy = mockBridge({
      "srv/pick_and_place/run": { accepted: true, message: "" },
    });
    const { getByTestId } = render(<PickAndPlacePanel />);

    fireEvent.click(getByTestId("pnp-build-world"));
    fireEvent.change(getByTestId("pnp-pick"), { target: { value: "cube" } });
    await act(async () => {
      fireEvent.click(getByTestId("pnp-run"));
    });

    const calls = spy.mock.calls.filter(
      (c) => String(c[0]) === "srv/pick_and_place/run",
    );
    expect(calls.length).toBe(1);
    expect((calls[0][1] as { build_world?: boolean }).build_world).toBe(true);
    expect(localStorage.getItem("pnp.buildWorld")).toBe("1");
    localStorage.removeItem("pnp.buildWorld"); // 같은 파일 뒤 테스트 오염 방지
  });

  it("월드 갱신 체크 시 voxel 셀렉터 노출 → 선택이 world_voxel_size 로 실행 + 기억", async () => {
    const spy = mockBridge({
      "srv/pick_and_place/run": { accepted: true, message: "" },
    });
    const { getByTestId, queryByTestId } = render(<PickAndPlacePanel />);

    // 갱신 off 면 voxel 셀렉터 숨김 (관련 없는 노브 노출 안 함)
    expect(queryByTestId("pnp-world-voxel")).toBeNull();
    fireEvent.click(getByTestId("pnp-build-world"));
    fireEvent.change(getByTestId("pnp-world-voxel"), { target: { value: "0.004" } });
    fireEvent.change(getByTestId("pnp-pick"), { target: { value: "cube" } });
    await act(async () => {
      fireEvent.click(getByTestId("pnp-run"));
    });

    const calls = spy.mock.calls.filter(
      (c) => String(c[0]) === "srv/pick_and_place/run",
    );
    expect((calls[0][1] as { world_voxel_size?: number }).world_voxel_size).toBe(
      0.004,
    );
    expect(localStorage.getItem("pnp.worldVoxelM")).toBe("0.004");
    localStorage.removeItem("pnp.buildWorld");
    localStorage.removeItem("pnp.worldVoxelM");
  });

  it("실행 거부 → 사유 표시 (침묵 금지)", async () => {
    mockBridge({
      "srv/pick_and_place/run": {
        accepted: false,
        message: "이미 실행 중 (pick_and_place)",
      },
    });
    const { getByTestId } = render(<PickAndPlacePanel />);
    fireEvent.change(getByTestId("pnp-pick"), { target: { value: "white cube" } });
    await act(async () => {
      fireEvent.click(getByTestId("pnp-run"));
    });
    expect(getByTestId("pnp-msg").textContent).toContain("이미 실행 중");
  });

  it("중지 → PICKANDPLACE_STOP, 실패 시 사유 표시", async () => {
    const spy = mockBridge({
      "srv/pick_and_place/stop": { ok: false, message: "실행 중인 run 없음" },
    });
    const { getByTestId } = render(<PickAndPlacePanel />);
    await act(async () => {
      fireEvent.click(getByTestId("pnp-stop"));
    });
    const calls = spy.mock.calls.filter((c) =>
      String(c[0]) === "srv/pick_and_place/stop",
    );
    expect(calls.length).toBe(1);
    expect(getByTestId("pnp-msg").textContent).toContain("실행 중인 run 없음");
  });

  it("pick 비면 실행 버튼 disabled (필수 param)", () => {
    mockBridge();
    const { getByTestId } = render(<PickAndPlacePanel />);
    expect((getByTestId("pnp-run") as HTMLButtonElement).disabled).toBe(true);
  });
});
