// WaypointPanel — 패널 → waypoint 서비스 wire 검증 (unit).
// 전체 stateful 흐름(teach→list 반영)은 e2e(waypoint.spec.ts)가 실 backend 로 검증.
// 여기선 클릭 → 올바른 ServiceKey + payload 로 callService 되는지 (RobotStatePanel
// 테스트 철학과 동일).

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render, waitFor } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { bridge } from "@/api/bridge";
import { useFrameworkStore, type ServiceEntry } from "@/framework/store";
import { WaypointPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";

/** key 별 최소 응답 — 패널이 res.data 를 읽으니 shape 맞춰줌. */
function respond(key: string): unknown {
  if (key.includes("list_group_members")) return { waypoints: [] };
  if (key.includes("list_groups")) return { groups: [] };
  if (key.endsWith("/list")) return { waypoints: [] };
  if (key.includes("/teach")) {
    return {
      accepted: true,
      waypoint: {
        id: 1,
        robot_id: ROBOT_ID,
        name: "home",
        joint_values: [],
        joint_names: [],
        created_at: "",
      },
    };
  }
  if (key.includes("/create_group")) {
    return { accepted: true, group: { id: 1, robot_id: ROBOT_ID, name: "search" } };
  }
  return { ok: true };
}

function mockBridge() {
  return vi
    .spyOn(bridge, "callService")
    // @ts-expect-error — 테스트 stub, 응답 shape 는 respond() 가 책임
    .mockImplementation(async (key, _req, opts) => {
      const wk = bridge.expand(key, (opts as { robotId?: string })?.robotId ?? ROBOT_ID);
      const entry: ServiceEntry = {
        success: true,
        message: "",
        data: respond(String(key)),
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
        <Route path="/robots/:id" element={<WaypointPanel />} />
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

describe("WaypointPanel", () => {
  it("패널 + 두 탭 렌더", async () => {
    mockBridge();
    const { getByTestId } = renderPanel();
    // getByTestId 는 없으면 throw → 존재 자체가 assert (jest-dom matcher 회피).
    expect(getByTestId("waypoint-panel")).toBeTruthy();
    expect(getByTestId("tab-library")).toBeTruthy();
    expect(getByTestId("tab-groups")).toBeTruthy();
  });

  it("티칭 저장 → WAYPOINT_TEACH 를 입력 이름으로 call", async () => {
    const spy = mockBridge();
    const { getByTestId } = renderPanel();

    act(() => {
      fireEvent.change(getByTestId("wp-name"), { target: { value: "search_left" } });
    });
    await act(async () => {
      fireEvent.click(getByTestId("wp-teach"));
    });

    await waitFor(() => {
      const calls = spy.mock.calls.filter((c) => String(c[0]).includes("/teach"));
      expect(calls.length).toBeGreaterThan(0);
      expect((calls[0][1] as { name: string }).name).toBe("search_left");
    });
  });

  // group 생성/멤버/reorder 인터랙션(radix Tabs 전환 포함)은 실 chromium 에서
  // 검증 — e2e/waypoint.spec.ts. happy-dom + fireEvent 는 radix tab 전환을
  // 재현 못 함(user-event 미설치). 여기선 패널 렌더 + teach wire 까지.
});
