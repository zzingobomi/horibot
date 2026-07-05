// frontend_v2 L4 — RobotTaskMode(NL PnP) e2e (mock backend + vite dev).
//
// 검증 invariant (frontend ↔ bridge WS ↔ llm/task 전 wire):
//   1. WS 연결 + PromptPanel + TaskProgressPanel 렌더
//   2. 명령 입력 → 파싱 → LLM_PARSE_COMMAND → (pick/place) 표시 (mock LLM = white cube/blue box)
//   3. 실행 → TASK_RUN → build_task → TASK_TREE(step 목록) + TASK_STATE(status) stream 반영
//
// 주의: mock backend 는 waypoint("search" group)·calibration(hand_eye) 이 비어 있어
// PnP 는 SearchWaypointGroup 에서 자연 실패(status=failed) — 이 e2e 는 "명령→파싱→
// 실행→진행 표시" **프론트↔백 wire** 검증이지 완주가 아님. 완주(정확도)는 실물
// waypoint 티칭 + 캘 + 하드웨어 (docs/backend_v2.md §17.5).
//
// 외부 의존 (실행 전 띄움):
//   - mock backend (port 8000): cd backend_v2 && uv run --no-sync python -m apps.main --host mock
//   - frontend vite (port 5174): cd frontend_v2 && pnpm dev

import { expect, test } from "@playwright/test";

const TASKS_PATH = "/tasks"; // 최상위 (host-level, robot-agnostic — 로봇 하위 아님)
const COMMAND = "흰색 큐브를 파란 상자에 둬";

test.describe("RobotTaskMode e2e (mock backend)", () => {
  test("WS 연결 + Prompt/TaskProgress 패널 렌더", async ({ page }) => {
    await page.goto(TASKS_PATH);
    // Sidebar 연결 인디케이터 = "online"/"offline" (connected 텍스트 아님).
    await expect(page.getByText("online", { exact: true })).toBeVisible({
      timeout: 5_000,
    });
    await expect(page.getByTestId("prompt-panel")).toBeVisible({
      timeout: 5_000,
    });
    await expect(page.getByTestId("task-progress-panel")).toBeVisible({
      timeout: 5_000,
    });
  });

  test("명령 → 파싱 → pick/place 표시", async ({ page }) => {
    await page.goto(TASKS_PATH);
    await expect(page.getByTestId("prompt-panel")).toBeVisible({
      timeout: 5_000,
    });

    await page.getByTestId("prompt-input").fill(COMMAND);
    await page.getByTestId("prompt-parse").click();

    // mock LLM 고정 파싱 → pick=white cube / place=blue box
    await expect(page.getByTestId("prompt-parsed")).toContainText("white cube", {
      timeout: 5_000,
    });
  });

  test("실행 → task tree(step) + status stream 반영", async ({ page }) => {
    await page.goto(TASKS_PATH);
    await expect(page.getByTestId("prompt-panel")).toBeVisible({
      timeout: 5_000,
    });

    await page.getByTestId("prompt-input").fill(COMMAND);
    await page.getByTestId("prompt-parse").click();
    await expect(page.getByTestId("prompt-parsed")).toContainText("white cube", {
      timeout: 5_000,
    });

    await page.getByTestId("prompt-run").click();

    // TASK_RUN → build_task → TASK_TREE 발행 → PnP 첫 step "open_gripper" 표시
    await expect(page.getByTestId("task-steps")).toContainText("open_gripper", {
      timeout: 5_000,
    });
    // runner 실행 → TASK_STATE status 가 idle 이탈 (mock 은 waypoint 없어 failed 로 귀결)
    await expect(page.getByTestId("task-status")).toHaveText(
      /running|failed|success/i,
      { timeout: 5_000 },
    );
  });
});
