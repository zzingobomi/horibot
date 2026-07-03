// frontend_v2 L4 — RobotAssetsMode(Waypoint) e2e (mock backend + vite dev).
//
// 검증 invariant (frontend ↔ bridge WS ↔ waypoint 서비스 ↔ DB 전 wire):
//   1. WS 연결 + WaypointPanel 렌더
//   2. 티칭 → WAYPOINT_TEACH (현재 joint 저장) → WAYPOINT_LIST 에 표시
//   3. group 생성 + 멤버 추가 → WAYPOINT_CREATE_GROUP / ADD_TO_GROUP / LIST_GROUP_MEMBERS
//
// 티칭은 backend WaypointModule 이 Motion.TcpState(rad) 를 캐시 → mock 은 motor+motion
// 이라 joint state 도착함 (teach accepted).
//
// 외부 의존 (실행 전 띄움):
//   - mock backend (port 8000): cd backend_v2 && uv run --no-sync python -m apps.main --host mock
//   - frontend vite (port 5174): cd frontend_v2 && pnpm dev

import { expect, test } from "@playwright/test";

const ASSETS_PATH = "/robots/so101_6dof_0/assets";

test.describe("RobotAssetsMode e2e (mock backend)", () => {
  test("WS 연결 + WaypointPanel 렌더", async ({ page }) => {
    await page.goto(ASSETS_PATH);
    await expect(page.getByText("connected", { exact: true })).toBeVisible({
      timeout: 5_000,
    });
    await expect(page.getByTestId("waypoint-panel")).toBeVisible({
      timeout: 5_000,
    });
  });

  test("티칭 → waypoint 목록에 표시", async ({ page }) => {
    await page.goto(ASSETS_PATH);
    await expect(page.getByTestId("waypoint-panel")).toBeVisible({
      timeout: 5_000,
    });
    const name = "e2e_wp";
    await page.getByTestId("wp-name").fill(name);
    await page.getByTestId("wp-teach").click();
    // TEACH → 현재 joint 저장 → LIST 새로고침 → 목록에 이름
    await expect(page.getByTestId("wp-list")).toContainText(name, {
      timeout: 5_000,
    });
  });

  test("group 생성 + 멤버 추가 → members 목록", async ({ page }) => {
    await page.goto(ASSETS_PATH);
    await expect(page.getByTestId("waypoint-panel")).toBeVisible({
      timeout: 5_000,
    });

    // waypoint 하나 보장
    await page.getByTestId("wp-name").fill("wp_a");
    await page.getByTestId("wp-teach").click();
    await expect(page.getByTestId("wp-list")).toContainText("wp_a", {
      timeout: 5_000,
    });

    // Groups 탭 → group 생성 → 선택
    await page.getByTestId("tab-groups").click();
    await page.getByTestId("wp-group-name").fill("grp");
    await page.getByTestId("wp-create-group").click();
    await page.getByTestId("wp-group-select").first().click();

    // addable 에서 wp_a 를 명시적으로 추가 (alphabetical first 아님 — 결정론)
    const addWpA = page.getByTestId("wp-add-member").filter({ hasText: "wp_a" });
    await expect(addWpA).toBeVisible({ timeout: 5_000 });
    await addWpA.click();
    await expect(page.getByTestId("wp-member-list")).toContainText("wp_a", {
      timeout: 5_000,
    });
  });
});
