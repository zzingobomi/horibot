import { defineConfig, devices } from "@playwright/test";

// frontend_v2.md §12.1 L4 — Playwright + mock backend e2e.
// 외부 사전 조건: 아래 두 서비스가 미리 떠 있어야 함.
//   - backend_v2 mock backend (port 8000): `cd backend_v2 && uv run --no-sync python -m apps.main --host mock`
//   - frontend_v2 vite dev (port 5174): `cd frontend_v2 && pnpm dev`
// CI 자동화 단계에서는 webServer 옵션으로 두 서비스를 자동 기동.

export default defineConfig({
  testDir: "./e2e",
  fullyParallel: false, // mock backend single-process — 직렬 실행
  retries: 0,
  workers: 1,
  reporter: "list",
  timeout: 20_000,
  expect: { timeout: 5_000 },
  use: {
    baseURL: "http://localhost:5174",
    trace: "on-first-retry",
    screenshot: "only-on-failure",
  },
  projects: [
    {
      name: "chromium",
      // hasTouch: true — button hold 시 mouse cascade (100ms pointerup auto fire)
      // 회피 + CDP Input.dispatchTouchEvent 사용. 사용자가 실 hardware 박을 때
      // 빠른 손가락 / touch device 동일 path.
      use: { ...devices["Desktop Chrome"], hasTouch: true },
    },
  ],
});
