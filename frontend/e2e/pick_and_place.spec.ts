// frontend L4 — Pick & Place task 페이지 e2e (mock backend + vite dev).
//
// UX 워크스루 (대원칙: 모든 상태에서 나갈 수 있고, 실패는 사유+다음 행동):
//   1. WS 연결 + PickAndPlacePanel + TaskProgressPanel 렌더
//   2. 자연어 → 파싱 → 폼 채움 (mock LLM = white cube/blue box)
//   3. 실행 → RUN accepted → TRACE 에 detect_pick 누적 → FAILED + 사유 표시
//      (mock 은 캘 없음 → detector 후보 0 → 자연 실패. 완주는 실물 하드웨어)
//   4. breakpoint(label) → 재실행 → PAUSED (detect_pick 직전 hold)
//   5. PAUSED 에서 [중지] → STOPPED (탈출구)
//   6. run 없는데 [중지] → 사유 표시 (침묵 금지)
//
// 외부 의존 (실행 전 띄움):
//   - mock backend (port 8000): cd backend && uv run --no-sync python -m apps.main --host mock
//   - frontend vite (port 5173): cd frontend && pnpm dev

import { expect, test } from "@playwright/test";

const PAGE_PATH = "/tasks/pick_and_place";
const COMMAND = "흰색 작고 네모난 큐브를 파란 상자에 둬";

async function gotoReady(page: import("@playwright/test").Page) {
  await page.goto(PAGE_PATH);
  // Sidebar 연결 인디케이터 = "online" — ws OPEN 대기 후 조작 (e2e 결정성).
  await expect(page.getByText("online", { exact: true })).toBeVisible({
    timeout: 5_000,
  });
  await expect(page.getByTestId("pnp-panel")).toBeVisible({ timeout: 5_000 });
  await expect(page.getByTestId("task-progress-panel")).toBeVisible({
    timeout: 5_000,
  });
}

test.describe("Pick & Place task 페이지 e2e (mock backend)", () => {
  test("명령 → 파싱 → 폼 채움 → 실행 → 진행(trace) + 실패 사유 표시", async ({
    page,
  }) => {
    await gotoReady(page);

    await page.getByTestId("pnp-input").fill(COMMAND);
    await page.getByTestId("pnp-parse").click();

    // mock LLM 고정 파싱 → 폼에 pick=white cube / place=blue box
    await expect(page.getByTestId("pnp-pick")).toHaveValue(/white/i, {
      timeout: 5_000,
    });

    await page.getByTestId("pnp-run").click();

    // RUN → runner 감독 실행 → TRACE 에 첫 primitive (detect_pick) 누적
    await expect(page.getByTestId("task-entries")).toContainText("detect_pick", {
      timeout: 10_000,
    });
    // mock 은 캘 없음 → 검출 0 → FAILED. 실패는 침묵이 아니라 사유+다음 행동.
    await expect(page.getByTestId("task-status")).toHaveText(/failed/i, {
      timeout: 10_000,
    });
    await expect(page.getByTestId("task-error")).toContainText("다시 실행", {
      timeout: 5_000,
    });
  });

  test("breakpoint → 재실행 PAUSED (hold) → [중지] = 탈출구", async ({ page }) => {
    await gotoReady(page);

    // 1차 실행 — trace 를 만들어 breakpoint 대상(label) 확보
    await page.getByTestId("pnp-pick").fill("white cube");
    await page.getByTestId("pnp-run").click();
    await expect(page.getByTestId("task-entries")).toContainText("detect_pick", {
      timeout: 10_000,
    });
    await expect(page.getByTestId("task-status")).toHaveText(/failed/i, {
      timeout: 10_000,
    });

    // detect_pick 에 breakpoint (dot 클릭) — runner 가 run 간 보존
    await page.getByTestId("task-entry-bp").first().click();

    // 2차 실행 → detect_pick 직전 hold = PAUSED
    await page.getByTestId("pnp-run").click();
    await expect(page.getByTestId("task-status")).toHaveText(/paused/i, {
      timeout: 10_000,
    });

    // PAUSED 에서도 [중지] 로 즉시 탈출 (모든 상태에서 나갈 수 있어야)
    await page.getByTestId("pnp-stop").click();
    await expect(page.getByTestId("task-status")).toHaveText(/stopped/i, {
      timeout: 10_000,
    });

    // run 이 없는데 중지 → 사유 표시 (침묵 금지)
    await page.getByTestId("pnp-stop").click();
    await expect(page.getByTestId("pnp-msg")).toContainText("없음", {
      timeout: 5_000,
    });
  });
});
