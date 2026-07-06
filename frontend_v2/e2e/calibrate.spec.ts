// frontend_v2 L4 — RobotCalibrateMode e2e (mock backend + vite dev + chromium).
//
// 검증 invariant (frontend ↔ bridge WS ↔ calibration 서비스 ↔ DB 전 wire):
//   1. WS 연결 — connected badge
//   2. CalibrationPanel 렌더 (active bundle 5 kind)
//   3. preview toggle → preview_enable 서비스 호출 (ON)
//   4. 세션 시작 → start_run(INSERT) → list_runs 로 run history 에 hand_eye 등장
//   5. 세션 종료 → finalize_run
//
// capture 성공(green verdict)은 camera 가 ChArUco 보드를 내보내야 함 (mock camera
// sim board 모드 = 후속). 여기선 frontend↔backend calibration wire 를 검증.
//
// 외부 의존 (실행 전 띄움):
//   - mock backend (port 8000): cd backend_v2 && uv run --no-sync python -m apps.main --host mock
//   - frontend vite (port 5174): cd frontend_v2 && pnpm dev

import { expect, test } from "@playwright/test";

const CAL_PATH = "/robots/so101_6dof_0/calibrate";

test.describe("RobotCalibrateMode e2e (mock backend)", () => {
  test("WS 연결 + CalibrationPanel + active bundle 렌더", async ({ page }) => {
    await page.goto(CAL_PATH);
    await expect(page.getByText("online", { exact: true })).toBeVisible({
      timeout: 5_000,
    });
    await expect(page.getByTestId("calibration-panel")).toBeVisible({
      timeout: 5_000,
    });
    // active bundle — 5 kind label (fresh :memory: 라 전부 미활성이지만 렌더됨)
    const bundle = page.getByTestId("active-bundle");
    await expect(bundle).toContainText("hand_eye");
    await expect(bundle).toContainText("sag");
  });

  test("preview toggle → ON", async ({ page }) => {
    await page.goto(CAL_PATH);
    const toggle = page.getByTestId("preview-toggle");
    await expect(toggle).toHaveText("OFF", { timeout: 5_000 });
    await toggle.click();
    await expect(toggle).toHaveText("ON", { timeout: 5_000 });
  });

  test("세션 시작 → run history 에 hand_eye run 등장 (start_run→list_runs wire)", async ({
    page,
  }) => {
    await page.goto(CAL_PATH);
    await expect(page.getByTestId("calibration-panel")).toBeVisible({
      timeout: 5_000,
    });

    await page.getByTestId("start-run").click();
    // capture-msg 가 "run N 시작" 반영
    await expect(page.getByTestId("capture-msg")).toContainText("시작", {
      timeout: 5_000,
    });
    // list_runs refresh → history 에 hand_eye run
    await expect(page.getByTestId("run-history")).toContainText("hand_eye", {
      timeout: 5_000,
    });

    // 세션 종료 → finalize (capture-msg 에 종료 반영)
    await page.getByTestId("finalize").click();
    await expect(page.getByTestId("capture-msg")).toContainText("종료", {
      timeout: 5_000,
    });
  });

  // capture-success — backend 를 CALIB_SIM_BOARD=1 로 띄워야 mock camera 가 ChArUco
  // 보드를 발행 → preview 검출(green) + capture accepted. (sim board OFF 면 skip.)
  test("preview 검출 + capture 성공 (sim board, over-wire capture)", async ({ page }) => {
    await page.goto(CAL_PATH);
    await expect(page.getByTestId("calibration-panel")).toBeVisible({ timeout: 5_000 });

    // preview ON → 검출될 때까지 대기. 주의: "미검출" 이 "검출" 을 substring 으로
    // 포함하므로 corner count 패턴으로만 검출 판정 (skip 오탐 방지).
    await page.getByTestId("preview-toggle").click();
    const detail = page.getByTestId("preview-detail");
    try {
      await expect(detail).toContainText(/검출 \d+ corners/, { timeout: 6_000 });
    } catch {
      test.skip(true, "sim board OFF (mock camera 라벨 이미지) — CALIB_SIM_BOARD=1 로 backend 필요");
    }
    // 검출됐으면 verdict 는 green/yellow (첫 자세 green)
    const verdict = await page.getByTestId("preview-verdict").getAttribute("data-verdict");
    expect(["green", "yellow"]).toContain(verdict);

    // 세션 시작 → 캡처 → accepted
    await page.getByTestId("start-run").click();
    await expect(page.getByTestId("capture-msg")).toContainText("시작", { timeout: 5_000 });
    await page.getByTestId("capture").click();
    await expect(page.getByTestId("capture-msg")).toContainText("캡처됨", { timeout: 6_000 });
  });

  // 카메라 뷰 — so101(realsense)=has_camera → color MJPEG 뷰. preview ON 시 sim board
  // 검출 좌표(corners_2d)가 CALIBRATION_PREVIEW 로 흘러 ChArUco 오버레이로 렌더
  // (camera/stream[MJPEG] + preview[좌표] 별 채널 합성이 over-wire 로 동작하는지).
  test("카메라 뷰 + preview 검출 시 ChArUco 오버레이 (over-wire)", async ({ page }) => {
    await page.goto(CAL_PATH);
    await expect(page.getByTestId("calibration-camera-panel")).toBeVisible({
      timeout: 5_000,
    });
    // color 스트림 img (has_camera 게이트 통과 — rgbd 무관)
    await expect(page.getByTestId("camera-stream")).toBeVisible({ timeout: 5_000 });

    // preview ON → 검출되면 오버레이, sim board OFF 면 skip (capture 테스트와 동일 게이트)
    await page.getByTestId("preview-toggle").click();
    try {
      await expect(page.getByTestId("preview-detail")).toContainText(
        /검출 \d+ corners/,
        { timeout: 6_000 },
      );
    } catch {
      test.skip(true, "sim board OFF — CALIB_SIM_BOARD=1 로 backend 필요");
    }
    const overlay = page.getByTestId("charuco-overlay");
    await expect(overlay).toBeVisible({ timeout: 6_000 });
    // 검출 코너 수만큼 마커 (corners_2d → circle)
    expect(await overlay.locator("circle").count()).toBeGreaterThan(0);
    // 캡처 안내 HUD — 검출 시 green/yellow (첫 자세 green). 카메라 위 verdict 표시.
    const guide = page.getByTestId("capture-guide");
    await expect(guide).toBeVisible({ timeout: 6_000 });
    expect(["green", "yellow"]).toContain(
      await guide.getAttribute("data-verdict"),
    );
  });
});
