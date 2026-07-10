// frontend L4 — RobotAssetsMode(Waypoint) e2e (mock backend + vite dev).
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
//   - mock backend (port 8000): cd backend && uv run --no-sync python -m apps.main --host mock
//   - frontend vite (port 5173): cd frontend && pnpm dev

import { expect, test } from "@playwright/test";

const ASSETS_PATH = "/robots/so101_6dof_0/assets";

test.describe("RobotAssetsMode e2e (mock backend)", () => {
  test("WS 연결 + WaypointPanel 렌더", async ({ page }) => {
    await page.goto(ASSETS_PATH);
    await expect(page.getByText("online", { exact: true })).toBeVisible({
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
    // mock 은 in-memory DB 를 프로세스 내내 공유(테스트 격리 없음)라 고정 이름은
    // 반복 실행 시 누적. run 마다 unique 이름으로 격리.
    const name = `e2e_wp_${Date.now()}`;
    // TEACH 는 backend WaypointModule 의 joint 캐시(Motion tcp stream)가 있어야
    // accepted — 페이지 로드 직후엔 아직 미도달일 수 있어 목록에 뜰 때까지 재시도
    // (cold-cache race, 실 사용은 캐시 데워진 뒤 티칭). 성공 시 name 필드가 비므로
    // 매 시도 re-fill.
    await expect(async () => {
      await page.getByTestId("wp-name").fill(name);
      await page.getByTestId("wp-teach").click();
      await expect(page.getByTestId("wp-list")).toContainText(name, {
        timeout: 1_500,
      });
    }).toPass({ timeout: 10_000 });
  });

  test("group 생성 + 멤버 추가 → members 목록", async ({ page }) => {
    await page.goto(ASSETS_PATH);
    await expect(page.getByTestId("waypoint-panel")).toBeVisible({
      timeout: 5_000,
    });

    // mock in-memory DB 는 프로세스 공유 → 고정 이름은 반복 실행 시 누적/오선택.
    // run 마다 unique 이름으로 격리 (그래야 방금 만든 group 이 fresh·empty 라
    // addable 에 wp 가 확실히 있고, .first() stale group 오선택도 없음).
    const wp = `wp_${Date.now()}`;
    const grp = `grp_${Date.now()}`;

    // waypoint 하나 보장 — teach cold-cache race 재시도
    await expect(async () => {
      await page.getByTestId("wp-name").fill(wp);
      await page.getByTestId("wp-teach").click();
      await expect(page.getByTestId("wp-list")).toContainText(wp, {
        timeout: 1_500,
      });
    }).toPass({ timeout: 10_000 });

    // Groups 탭 → group 생성 → 방금 만든 group 을 **이름으로** 선택 (.first() 는
    // 누적된 stale group 을 골라 addable 이 어긋나던 원인).
    await page.getByTestId("tab-groups").click();
    await page.getByTestId("wp-group-name").fill(grp);
    await page.getByTestId("wp-create-group").click();
    await page.getByTestId("wp-group-select").filter({ hasText: grp }).click();

    // fresh empty group → addable 에 방금 teach 한 wp 존재. 추가 → members 반영.
    const addWp = page.getByTestId("wp-add-member").filter({ hasText: wp });
    await expect(addWp).toBeVisible({ timeout: 5_000 });
    await addWp.click();
    await expect(page.getByTestId("wp-member-list")).toContainText(wp, {
      timeout: 5_000,
    });
  });
});
