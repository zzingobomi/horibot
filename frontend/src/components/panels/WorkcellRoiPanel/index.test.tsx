// WorkcellRoiPanel — 패널 → shared_config 서비스 wire 검증 (unit, WaypointPanel
// 테스트 철학과 동일: 클릭 → 올바른 ServiceKey + payload).
//
// 뒤집으면 회귀: 저장이 draft 아닌 다른 값 전송 / 저장 실패가 draft 를 날림
// (편집 유실 = 탈출구 부재) / 미설정 robot 에 "만들기" 진입점이 없음 /
// 숫자 입력이 min/max 역전을 그대로 통과.

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render, waitFor } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { bridge } from "@/api/bridge";
import { useFrameworkStore, type ServiceEntry } from "@/framework/store";
import { RobotProvider } from "@/components/shared/robotOwnership";
import { useWorkcellRoiStore } from "@/stores/workcellRoiStore";
import type { WorkcellRoi } from "@/api/generated/contract";
import { WorkcellRoiPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";

const ROI: WorkcellRoi = {
  x_min: 0.0,
  x_max: 0.35,
  y_min: -0.3,
  y_max: 0.3,
  z_min: -0.05,
  z_max: 0.35,
};

// 테스트별 주입 — snapshot 응답의 robots dict / set 성공 여부
let snapshotRobots: Record<string, WorkcellRoi> = {};
let setFails = false;

function mockBridge() {
  return vi
    .spyOn(bridge, "callService")
    // @ts-expect-error — 테스트 stub, 응답 shape 는 아래가 책임
    .mockImplementation(async (key, req, opts) => {
      const k = String(key);
      let entry: ServiceEntry;
      if (k.includes("snapshot_workcell")) {
        entry = {
          success: true,
          message: "",
          data: { robots: snapshotRobots },
          timestamp: Date.now(),
          pending: false,
        };
      } else if (k.includes("set_workcell")) {
        entry = setFails
          ? {
              success: false,
              message: "disk full",
              data: null,
              timestamp: Date.now(),
              pending: false,
            }
          : {
              success: true,
              message: "",
              data: { roi: (req as { roi: WorkcellRoi }).roi },
              timestamp: Date.now(),
              pending: false,
            };
      } else {
        entry = {
          success: true,
          message: "",
          data: {},
          timestamp: Date.now(),
          pending: false,
        };
      }
      const wk = bridge.serviceCacheKey(k, (opts as { robotId?: string })?.robotId);
      useFrameworkStore.getState().setServiceData(wk, entry);
      return entry;
    });
}

function renderPanel() {
  return render(
    <MemoryRouter initialEntries={[`/robots/${ROBOT_ID}`]}>
      <Routes>
        <Route
          path="/robots/:id"
          element={
            <RobotProvider robotId={ROBOT_ID}>
              <WorkcellRoiPanel />
            </RobotProvider>
          }
        />
      </Routes>
    </MemoryRouter>,
  );
}

beforeEach(() => {
  snapshotRobots = {};
  setFails = false;
  useWorkcellRoiStore.setState({ saved: {}, drafts: {}, activeFace: {} });
  useFrameworkStore.setState({
    topicData: {},
    serviceData: {},
    bridgeConnected: true,
  });
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("WorkcellRoiPanel", () => {
  it("snapshot 로드 → 필드에 저장값 표시 + 저장 버튼 비활성 (dirty 아님)", async () => {
    snapshotRobots = { [ROBOT_ID]: ROI };
    mockBridge();
    const { getByTestId } = renderPanel();
    await waitFor(() => {
      expect((getByTestId("roi-x_max") as HTMLInputElement).value).toBe("0.35");
    });
    expect((getByTestId("roi-save") as HTMLButtonElement).disabled).toBe(true);
  });

  it("미설정 robot → 'ROI 만들기' 진입점 → draft 생성 (미저장 표시)", async () => {
    mockBridge();
    const { getByTestId } = renderPanel();
    await waitFor(() => expect(getByTestId("roi-create")).toBeTruthy());
    act(() => {
      fireEvent.click(getByTestId("roi-create"));
    });
    expect(getByTestId("roi-dirty")).toBeTruthy(); // saved 없음 = 새 ROI = dirty
    expect((getByTestId("roi-save") as HTMLButtonElement).disabled).toBe(false);
  });

  it("필드 수정 → 저장 = SET_WORKCELL(robot_id, draft) + dirty 해제", async () => {
    snapshotRobots = { [ROBOT_ID]: ROI };
    const spy = mockBridge();
    const { getByTestId, queryByTestId } = renderPanel();
    await waitFor(() => {
      expect((getByTestId("roi-x_max") as HTMLInputElement).value).toBe("0.35");
    });

    act(() => {
      fireEvent.change(getByTestId("roi-x_max"), { target: { value: "0.4" } });
    });
    expect(getByTestId("roi-dirty")).toBeTruthy();
    await act(async () => {
      fireEvent.click(getByTestId("roi-save"));
    });

    await waitFor(() => {
      const calls = spy.mock.calls.filter((c) =>
        String(c[0]).includes("set_workcell"),
      );
      expect(calls.length).toBe(1);
      const req = calls[0][1] as { robot_id: string; roi: WorkcellRoi };
      expect(req.robot_id).toBe(ROBOT_ID);
      expect(req.roi.x_max).toBeCloseTo(0.4);
    });
    expect(queryByTestId("roi-dirty")).toBeNull(); // 응답값 = saved = draft
  });

  it("저장 실패 → draft 유지 + 사유 표시 (편집 유실 금지)", async () => {
    snapshotRobots = { [ROBOT_ID]: ROI };
    setFails = true;
    mockBridge();
    const { getByTestId } = renderPanel();
    await waitFor(() => {
      expect((getByTestId("roi-x_max") as HTMLInputElement).value).toBe("0.35");
    });

    act(() => {
      fireEvent.change(getByTestId("roi-x_max"), { target: { value: "0.4" } });
    });
    await act(async () => {
      fireEvent.click(getByTestId("roi-save"));
    });

    await waitFor(() => {
      expect(getByTestId("roi-status").textContent).toContain("disk full");
    });
    expect((getByTestId("roi-x_max") as HTMLInputElement).value).toBe("0.4");
    expect(getByTestId("roi-dirty")).toBeTruthy();
  });

  it("min 입력이 max 를 넘으면 clamp (역전 ROI 는 wire 로 못 나감)", async () => {
    snapshotRobots = { [ROBOT_ID]: ROI };
    mockBridge();
    const { getByTestId } = renderPanel();
    await waitFor(() => {
      expect((getByTestId("roi-x_min") as HTMLInputElement).value).toBe("0");
    });
    act(() => {
      fireEvent.change(getByTestId("roi-x_min"), { target: { value: "0.9" } });
    });
    const v = Number((getByTestId("roi-x_min") as HTMLInputElement).value);
    expect(v).toBeLessThan(0.35); // x_max(0.35) − MIN_SPAN 에서 정지
  });

  it("되돌리기 — draft = saved 복원", async () => {
    snapshotRobots = { [ROBOT_ID]: ROI };
    mockBridge();
    const { getByTestId, queryByTestId } = renderPanel();
    await waitFor(() => {
      expect((getByTestId("roi-x_max") as HTMLInputElement).value).toBe("0.35");
    });
    act(() => {
      fireEvent.change(getByTestId("roi-x_max"), { target: { value: "0.5" } });
    });
    expect(getByTestId("roi-dirty")).toBeTruthy();
    act(() => {
      fireEvent.click(getByTestId("roi-revert"));
    });
    expect(queryByTestId("roi-dirty")).toBeNull();
    expect((getByTestId("roi-x_max") as HTMLInputElement).value).toBe("0.35");
  });
});
