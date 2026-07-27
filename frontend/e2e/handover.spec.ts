// frontend L4 — Handover task 페이지 e2e (mock backend + vite dev).
//
// UX 워크스루 (대원칙: 모든 상태에서 나갈 수 있고, 실패는 사유+다음 행동):
//   1. WS 연결 + HandoverPanel + HandoverProgressPanel 렌더 + 참여 robot 표시
//   2. 실행 전 정적 프리뷰 — handover 시나리오 step 구조 (backend PREVIEW)
//   3. stop_before_receive 기본 ON 확인 → 실행 → RUN accepted → 진행 후
//      FAILED + 사유 표시 (mock 은 자산/캘이 비어 자연 실패 — 완주는 실물)
//   4. run 없는데 [중지] → 사유 표시 (침묵 금지)
//
// 외부 의존 (실행 전 띄움):
//   - mock backend (port 8000): cd backend && uv run --no-sync python -m apps.main --host mock
//   - frontend vite (port 5173): cd frontend && pnpm dev

import { expect, test } from "@playwright/test";

const PAGE_PATH = "/tasks/handover";

async function gotoReady(page: import("@playwright/test").Page) {
  await page.goto(PAGE_PATH);
  await expect(page.getByText("online", { exact: true })).toBeVisible({
    timeout: 5_000,
  });
  await expect(page.getByTestId("handover-panel")).toBeVisible({
    timeout: 5_000,
  });
  await expect(page.getByTestId("task-progress-panel")).toBeVisible({
    timeout: 5_000,
  });
}

test.describe("Handover task 페이지 e2e (mock backend)", () => {
  test("페이지 렌더 → 참여 robot + 정적 프리뷰 → 실행 → 실패 사유 표시", async ({
    page,
  }) => {
    await gotoReady(page);

    // 참여 robot 명부 (LIST_ROBOTS 계약) — giver → receiver 표시
    await expect(page.getByTestId("handover-robots")).toContainText("omx_f_0", {
      timeout: 5_000,
    });

    // 실행 전 정적 프리뷰 — handover 시나리오 골격이 보인다 (PREVIEW 계약)
    await expect(page.getByTestId("task-entries")).toContainText(
      "plan_omx_present",
      { timeout: 10_000 },
    );

    // stop_before_receive 기본 ON (실물 검증 1단계 프로토콜)
    await expect(
      page.getByTestId("handover-stop-before-receive"),
    ).toBeChecked();

    await page.getByTestId("handover-run").click();

    // mock 은 자산(waypoint/캘)이 비어 자연 실패 — 실패는 사유+다음 행동 표시
    await expect(page.getByTestId("task-status")).toHaveText(/failed/i, {
      timeout: 10_000,
    });
    await expect(page.getByTestId("task-error")).toContainText("하세요", {
      timeout: 5_000,
    });
  });

  test("run 없는데 [중지] → 사유 표시 (침묵 금지)", async ({ page }) => {
    await gotoReady(page);
    await page.getByTestId("handover-stop").click();
    await expect(page.getByTestId("handover-msg")).toContainText("없", {
      timeout: 5_000,
    });
  });
});
