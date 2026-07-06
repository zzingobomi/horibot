// frontend_v2 L4 — RobotScanMode e2e (mock backend + vite dev + chromium headed).
//
// 검증 invariant (frontend ↔ bridge WS ↔ scene3d/scan 서비스 ↔ DB 전 wire):
//   1. WS 연결 — connected badge
//   2. ScanPanel 렌더
//   3. live 토글 → SCENE3D_SET_STREAM (ON)
//   4. 새 세션 → SCAN_NEW_SESSION (session-current 에 #id)
//   5. 캡처 x2 → SCAN_CAPTURE + SCAN_LIST_SCANS (capture 버튼 count 증가)
//
// build(TSDF)/mesh 렌더 정확성은 backend e2e (test_scan_e2e.py) 가 검증 — 여기선
// 프론트→백엔드 서비스 wire 만. mock 은 hand_eye 없어 build reject 라 e2e 제외.
//
// 외부 의존 (실행 전 띄움):
//   - mock backend (port 8000): cd backend_v2 && uv run --no-sync python -m apps.main --host mock
//   - frontend vite (port 5174): cd frontend_v2 && pnpm dev

import { expect, test } from "@playwright/test";

const SCAN_PATH = "/robots/so101_6dof_0/scan";

test.describe("RobotScanMode e2e (mock backend)", () => {
  test("WS 연결 + ScanPanel 렌더", async ({ page }) => {
    await page.goto(SCAN_PATH);
    await expect(page.getByText("online", { exact: true })).toBeVisible({
      timeout: 5_000,
    });
    await expect(page.getByTestId("scan-panel")).toBeVisible({ timeout: 5_000 });
  });

  test("live 토글 → ON", async ({ page }) => {
    await page.goto(SCAN_PATH);
    const toggle = page.getByTestId("live-toggle");
    await expect(toggle).toHaveText("OFF", { timeout: 5_000 });
    await toggle.click();
    await expect(toggle).toHaveText("ON", { timeout: 5_000 });
  });

  test("세션 시작 + 캡처 x2 → scan count 증가", async ({ page }) => {
    await page.goto(SCAN_PATH);
    await expect(page.getByTestId("scan-panel")).toBeVisible({ timeout: 5_000 });

    // 새 세션 → session-current 에 #id
    await page.getByTestId("new-session").click();
    await expect(page.getByTestId("session-current")).toContainText("#", {
      timeout: 5_000,
    });

    // 캡처 버튼은 세션 후 활성 — 2회 캡처 (scene3d snapshot + blob 저장 wire)
    const capture = page.getByTestId("capture");
    await expect(capture).toBeEnabled({ timeout: 5_000 });
    await capture.click();
    await expect(capture).toContainText("캡처 (1)", { timeout: 10_000 });
    await capture.click();
    await expect(capture).toContainText("캡처 (2)", { timeout: 10_000 });
  });

  // 카메라 뷰 — scan 은 3D 점군만이 아니라 color 카메라도 봐야 어디를 비추는지 앎.
  // 범용 CameraPanel (오버레이 없음 — ChArUco 마커는 calibration 전용).
  test("카메라 뷰 렌더 (color, 오버레이 없음)", async ({ page }) => {
    await page.goto(SCAN_PATH);
    await expect(page.getByTestId("camera-panel")).toBeVisible({ timeout: 5_000 });
    await expect(page.getByTestId("camera-stream")).toBeVisible({ timeout: 5_000 });
    // scan 카메라는 ChArUco 오버레이 없음 (calibration 전용 관심사)
    await expect(page.getByTestId("charuco-overlay")).toHaveCount(0);
  });
});
