// CalibrationCameraPanel — CALIBRATION_PREVIEW(stream) → ChArUcoOverlay wire 검증.
// CameraView(color) + 마커 오버레이 합성이 실제로 preview 좌표를 그리는가 (프론트↔백 wire).

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, render, waitFor } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { _resetResourceCache } from "@/framework/resource";
import { useFrameworkStore } from "@/framework/store";
import { CalibrationCameraPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";
const PREVIEW_WIRE = `stream/calibration/${ROBOT_ID}/preview`;

beforeEach(() => {
  _resetResourceCache();
  useFrameworkStore.setState({ topicData: {}, serviceData: {}, bridgeConnected: true });
  vi.spyOn(globalThis, "fetch").mockResolvedValue(
    new Response(
      JSON.stringify({
        robots: [
          { id: ROBOT_ID, type: "so101_6dof", capabilities: [], has_camera: true },
        ],
        default: ROBOT_ID,
      }),
      { status: 200, headers: { "Content-Type": "application/json" } },
    ),
  );
});
afterEach(() => vi.restoreAllMocks());

function renderPanel() {
  return render(
    <MemoryRouter initialEntries={[`/robots/${ROBOT_ID}`]}>
      <Routes>
        <Route path="/robots/:id" element={<CalibrationCameraPanel />} />
      </Routes>
    </MemoryRouter>,
  );
}

describe("CalibrationCameraPanel", () => {
  it("preview stream 의 corners_2d → 오버레이 마커로 렌더", async () => {
    const { getByTestId } = renderPanel();
    // 카메라 뷰(has_camera=true) 뜰 때까지
    await waitFor(() => expect(getByTestId("camera-stream")).toBeTruthy());

    act(() => {
      useFrameworkStore.getState().setTopicData(PREVIEW_WIRE, {
        robot_id: ROBOT_ID,
        seq: 1,
        timestamp_unix: 0,
        detected: true,
        corner_count: 2,
        verdict: "green",
        corners_2d: [[10, 20], [30, 40]],
        image_width: 640,
        image_height: 480,
      });
    });

    await waitFor(() => {
      const svg = getByTestId("charuco-overlay");
      expect(svg.querySelectorAll("circle").length).toBe(2);
    });
    // 캡처 안내 HUD 도 같은 preview 로 렌더 (verdict/tilt 카메라 위 표시)
    const guide = getByTestId("capture-guide");
    expect(guide.getAttribute("data-verdict")).toBe("green");
    expect(guide.textContent).toContain("지금 캡처 OK");
  });

  it("preview 아직 없음 → 오버레이 없이 카메라 뷰만", async () => {
    const { getByTestId, queryByTestId } = renderPanel();
    await waitFor(() => expect(getByTestId("camera-stream")).toBeTruthy());
    expect(queryByTestId("charuco-overlay")).toBeNull();
  });
});
