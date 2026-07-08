// WaypointPanel — 패널 → waypoint 서비스 wire 검증 (unit).
// 전체 stateful 흐름(teach→list 반영)은 e2e(waypoint.spec.ts)가 실 backend 로 검증.
// 여기선 클릭 → 올바른 ServiceKey + payload 로 callService 되는지 (RobotStatePanel
// 테스트 철학과 동일).

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render, waitFor } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { bridge } from "@/api/bridge";
import { useFrameworkStore, type ServiceEntry } from "@/framework/store";
import { RobotProvider } from "@/components/shared/robotOwnership";
import { WaypointPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";

// 테스트별 주입 — 라이브러리 목록에 미리 waypoint 를 심어 행 인터랙션(이동/이름변경)
// 을 렌더링. beforeEach 에서 초기화.
let listWaypoints: unknown[] = [];

/** key 별 최소 응답 — 패널이 res.data 를 읽으니 shape 맞춰줌. */
function respond(key: string): unknown {
  if (key.includes("list_group_members")) return { waypoints: [] };
  if (key.includes("list_groups")) return { groups: [] };
  if (key.endsWith("/list")) return { waypoints: listWaypoints };
  if (key.includes("/move_j")) return { accepted: true };
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
        <Route
          path="/robots/:id"
          element={
            <RobotProvider robotId={ROBOT_ID}>
              <WaypointPanel />
            </RobotProvider>
          }
        />
      </Routes>
    </MemoryRouter>,
  );
}

beforeEach(() => {
  listWaypoints = [];
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
      const req = calls[0][1] as { robot_id: string; name: string };
      expect(req.name).toBe("search_left");
      // robot-agnostic 계약 — robot_id 는 req 필드 (키 치환 아님)
      expect(req.robot_id).toBe(ROBOT_ID);
    });
  });

  it("이동 → MOTION_MOVE_J 를 waypoint joint_values 로 call", async () => {
    listWaypoints = [
      {
        id: 7,
        robot_id: ROBOT_ID,
        name: "home",
        joint_values: [0.1, 0.2, 0.3, 0, 0, 0],
        joint_names: [],
        created_at: "",
      },
    ];
    const spy = mockBridge();
    const { getByTestId } = renderPanel();

    await waitFor(() => expect(getByTestId("wp-goto")).toBeTruthy());
    await act(async () => {
      fireEvent.click(getByTestId("wp-goto"));
    });

    await waitFor(() => {
      const calls = spy.mock.calls.filter((c) => String(c[0]).includes("/move_j"));
      expect(calls.length).toBeGreaterThan(0);
      const req = calls[0][1] as { target_joints: number[] };
      expect(req.target_joints).toEqual([0.1, 0.2, 0.3, 0, 0, 0]);
    });
  });

  it("이름변경 → 취소 시 edit 모드 종료 (rename call 없음)", async () => {
    listWaypoints = [
      {
        id: 7,
        robot_id: ROBOT_ID,
        name: "home",
        joint_values: [],
        joint_names: [],
        created_at: "",
      },
    ];
    const spy = mockBridge();
    const { getByTestId, queryByTestId } = renderPanel();

    await waitFor(() => expect(getByTestId("wp-rename")).toBeTruthy());
    act(() => {
      fireEvent.click(getByTestId("wp-rename"));
    });
    // edit 모드 진입 — 입력 + 취소 노출
    expect(getByTestId("wp-edit-name")).toBeTruthy();
    act(() => {
      fireEvent.click(getByTestId("wp-rename-cancel"));
    });
    // view 모드 복귀 — 입력 사라짐, rename 서비스 미호출
    expect(queryByTestId("wp-edit-name")).toBeNull();
    expect(spy.mock.calls.filter((c) => String(c[0]).includes("/rename")).length).toBe(0);
  });

  // group 생성/멤버/reorder 인터랙션(radix Tabs 전환 포함)은 실 chromium 에서
  // 검증 — e2e/waypoint.spec.ts. happy-dom + fireEvent 는 radix tab 전환을
  // 재현 못 함(user-event 미설치). 여기선 패널 렌더 + teach/goto/rename wire 까지.
});
