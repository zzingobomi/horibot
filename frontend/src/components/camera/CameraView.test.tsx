// CameraView — has_camera 게이팅 + 스트림 URL(SSOT=BASE_URL) 검증.
// color MJPEG 실 로드는 실 hardware (집) — 여기선 "게이트 + URL 조립 + children 슬롯".

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { render, waitFor } from "@testing-library/react";
import { _resetResourceCache } from "@/framework/resource";
import { BASE_URL } from "@/constants";
import { CameraView } from "./CameraView";

const ROBOT_ID = "so101_6dof_0";

function mockRobots(hasCamera: boolean) {
  vi.spyOn(globalThis, "fetch").mockResolvedValue(
    new Response(
      JSON.stringify({
        robots: [
          { id: ROBOT_ID, type: "so101_6dof", capabilities: [], has_camera: hasCamera },
        ],
        default: ROBOT_ID,
      }),
      { status: 200, headers: { "Content-Type": "application/json" } },
    ),
  );
}

beforeEach(() => _resetResourceCache());
afterEach(() => vi.restoreAllMocks());

describe("CameraView", () => {
  it("has_camera=true → camera/stream img (URL = BASE_URL SSOT)", async () => {
    mockRobots(true);
    const { getByTestId } = render(<CameraView robotId={ROBOT_ID} />);
    await waitFor(() => {
      const img = getByTestId("camera-stream") as HTMLImageElement;
      expect(img.getAttribute("src")).toBe(
        `${BASE_URL}/robots/${ROBOT_ID}/camera/stream`,
      );
    });
  });

  it("has_camera=false → 스트림 img 없이 '카메라 없음' 안내", async () => {
    mockRobots(false);
    const { getByTestId, queryByTestId } = render(<CameraView robotId={ROBOT_ID} />);
    await waitFor(() => {
      expect(getByTestId("camera-view").getAttribute("data-has-camera")).toBe("false");
    });
    expect(queryByTestId("camera-stream")).toBeNull();
  });

  it("children(오버레이) 을 이미지 위에 얹음", async () => {
    mockRobots(true);
    const { getByTestId } = render(
      <CameraView robotId={ROBOT_ID}>
        <div data-testid="overlay-slot" />
      </CameraView>,
    );
    await waitFor(() => expect(getByTestId("camera-stream")).toBeTruthy());
    expect(getByTestId("overlay-slot")).toBeTruthy();
  });
});
