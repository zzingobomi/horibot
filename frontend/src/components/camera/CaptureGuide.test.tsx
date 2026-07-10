// CaptureGuide — preview verdict/tilt → 카메라 위 캡처 안내 HUD (pure).
// 판정은 backend SSOT — 프론트는 verdict 를 라벨로 매핑만 (임계값 재derive X) 검증.

import { describe, expect, it } from "vitest";
import { render } from "@testing-library/react";
import { CaptureGuide } from "./CaptureGuide";
import type { CalibrationPreview } from "@/api/generated/contract";

function preview(over: Partial<CalibrationPreview>): CalibrationPreview {
  return {
    robot_id: "so101_6dof_0",
    seq: 1,
    timestamp_unix: 0,
    detected: true,
    corner_count: 10,
    verdict: "green",
    reasons: [],
    corners_2d: [],
    image_width: 640,
    image_height: 480,
    tilt_deg: 45,
    ...over,
  };
}

describe("CaptureGuide", () => {
  it("green → '지금 캡처 OK' + tilt 표시", () => {
    const { getByTestId } = render(<CaptureGuide preview={preview({ verdict: "green" })} />);
    const guide = getByTestId("capture-guide");
    expect(guide.getAttribute("data-verdict")).toBe("green");
    expect(guide.textContent).toContain("지금 캡처 OK");
    expect(getByTestId("capture-guide-tilt").textContent).toContain("45");
  });

  it("red → '캡처 불가' + 이유 표시", () => {
    const { getByTestId } = render(
      <CaptureGuide preview={preview({ verdict: "red", detected: false, reasons: ["보드 미검출"], tilt_deg: null })} />,
    );
    const guide = getByTestId("capture-guide");
    expect(guide.getAttribute("data-verdict")).toBe("red");
    expect(guide.textContent).toContain("캡처 불가");
    expect(guide.textContent).toContain("보드 미검출");
  });

  it("tilt 없으면(미검출) tilt readout 생략", () => {
    const { queryByTestId } = render(
      <CaptureGuide preview={preview({ tilt_deg: null })} />,
    );
    expect(queryByTestId("capture-guide-tilt")).toBeNull();
  });

  it("preview 없음 → null", () => {
    const { queryByTestId } = render(<CaptureGuide preview={null} />);
    expect(queryByTestId("capture-guide")).toBeNull();
  });
});
