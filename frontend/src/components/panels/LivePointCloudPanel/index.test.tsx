// LivePointCloudPanel — robot FK 캘 상태 배지 wire 검증 (2026-07-07 표면화).
// TcpState.calibration_applied/stale (backend motion Mirror 소비 결과) →
// 4 상태 배지: 대기 / 무보정(red) / 적용됨(green) / 변경-재시작필요(amber).
// "무보정으로 조용히 돈다" 차단이 목적 — red 케이스가 핵심.

import { beforeEach, describe, expect, it, vi } from "vitest";
import { render } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { useFrameworkStore } from "@/framework/store";
import { RobotProvider } from "@/components/shared/robotOwnership";
import { LivePointCloudPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";
const TCP_WIRE = `stream/motion/${ROBOT_ID}/tcp_state`;

function seedTcp(extra: Record<string, unknown> | null) {
  useFrameworkStore.setState({
    topicData:
      extra === null
        ? {}
        : {
            [TCP_WIRE]: {
              robot_id: ROBOT_ID,
              seq: 1,
              timestamp_unix: 1.0,
              position: [0.1, 0, 0.2],
              quaternion: [0, 0, 0, 1],
              joint_names: [],
              joints: [],
              ...extra,
            },
          },
    serviceData: {},
    bridgeConnected: false, // useMirror fetch 억제 — 배지 검증에 캘 번들 불필요
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
              <LivePointCloudPanel />
            </RobotProvider>
          }
        />
      </Routes>
    </MemoryRouter>,
  );
}

describe("LivePointCloudPanel — robot FK 캘 배지", () => {
  beforeEach(() => {
    vi.restoreAllMocks();
  });

  it("TCP stream 없음 → '대기 중'", () => {
    seedTcp(null);
    const { getByTestId } = renderPanel();
    expect(getByTestId("fk-calib-status").textContent).toContain("대기 중");
  });

  it("calibration_applied=false → 무보정 경고 (silent degradation 차단)", () => {
    seedTcp({ calibration_applied: false, calibration_stale: false });
    const { getByTestId } = renderPanel();
    expect(getByTestId("fk-calib-status").textContent).toContain("무보정");
  });

  it("calibration_applied=true → 적용됨", () => {
    seedTcp({ calibration_applied: true, calibration_stale: false });
    const { getByTestId } = renderPanel();
    expect(getByTestId("fk-calib-status").textContent).toContain("캘 적용됨");
  });

  it("calibration_stale=true → 재시작 필요 (applied 여부보다 우선)", () => {
    seedTcp({ calibration_applied: true, calibration_stale: true });
    const { getByTestId } = renderPanel();
    expect(getByTestId("fk-calib-status").textContent).toContain("재시작 필요");
  });
});
