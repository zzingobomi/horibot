// ChArUcoOverlay — corners_2d → SVG 마커 렌더 (pure, backend 무관).
// 좌표계 SSOT = 원본 프레임(viewBox). 스케일은 SVG preserveAspectRatio 가 흡수 —
// 여기선 "코너 개수/좌표/verdict 색이 그대로 원본 픽셀로 emit 되는가" 검증.

import { describe, expect, it } from "vitest";
import { render } from "@testing-library/react";
import { ChArUcoOverlay } from "./ChArUcoOverlay";
import type { CalibrationPreview } from "@/api/generated/contract";

function preview(over: Partial<CalibrationPreview>): CalibrationPreview {
  return {
    robot_id: "so101_6dof_0",
    seq: 1,
    timestamp_unix: 0,
    detected: true,
    corner_count: 0,
    verdict: "green",
    corners_2d: [],
    image_width: 640,
    image_height: 480,
    ...over,
  };
}

describe("ChArUcoOverlay", () => {
  it("corners_2d → 코너 개수만큼 circle, 원본 픽셀 좌표 그대로", () => {
    const { getByTestId } = render(
      <ChArUcoOverlay
        preview={preview({ corners_2d: [[100, 200], [300, 400], [50, 60]] })}
      />,
    );
    const svg = getByTestId("charuco-overlay");
    // 좌표계 = 원본 프레임 (viewBox) — 스케일 흡수의 SSOT
    expect(svg.getAttribute("viewBox")).toBe("0 0 640 480");
    expect(svg.getAttribute("preserveAspectRatio")).toBe("xMidYMid meet");
    const circles = svg.querySelectorAll("circle");
    expect(circles.length).toBe(3);
    expect(circles[0].getAttribute("cx")).toBe("100");
    expect(circles[0].getAttribute("cy")).toBe("200");
  });

  it("verdict → 마커 색", () => {
    const { getByTestId } = render(
      <ChArUcoOverlay preview={preview({ corners_2d: [[1, 1]], verdict: "red" })} />,
    );
    const circle = getByTestId("charuco-overlay").querySelector("circle");
    expect(circle?.getAttribute("stroke")).toBe("#ef4444");
  });

  it("검출 코너 0 → 아무것도 안 그림 (null)", () => {
    const { queryByTestId } = render(
      <ChArUcoOverlay preview={preview({ corners_2d: [] })} />,
    );
    expect(queryByTestId("charuco-overlay")).toBeNull();
  });

  it("원본 크기 없음 → null (스케일 기준 없어 좌표 못 맞춤)", () => {
    const { queryByTestId } = render(
      <ChArUcoOverlay
        preview={preview({ corners_2d: [[1, 1]], image_width: null, image_height: null })}
      />,
    );
    expect(queryByTestId("charuco-overlay")).toBeNull();
  });

  it("preview 없음(null) → null", () => {
    const { queryByTestId } = render(<ChArUcoOverlay preview={null} />);
    expect(queryByTestId("charuco-overlay")).toBeNull();
  });
});
