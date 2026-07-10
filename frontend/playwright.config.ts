import { defineConfig, devices } from "@playwright/test";

// frontend.md §12.1 L4 — Playwright + mock backend e2e.
// 외부 사전 조건: 아래 두 서비스가 미리 떠 있어야 함.
//   - mock backend (port 8000): `cd backend && uv run --no-sync python -m apps.main --host mock`
//   - frontend vite dev (port 5173): `cd frontend && pnpm dev`
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
    baseURL: "http://localhost:5173",
    trace: "on-first-retry",
    screenshot: "only-on-failure",
    // headless 기본 SwiftShader(소프트웨어 렌더)는 R3F 3D 씬 렌더로 메인스레드를
    // 굶겨 setInterval(50Hz jog) 이 ~10Hz 로 밀림 → jog publish rate < backend
    // IDLE_RESET(0.2s) → 모터 안 움직임. headed(실 GPU)면 메인스레드 자유 →
    // 50Hz 회복 (2026-07-01 측정 — frontend.md §12.1).
    headless: false,
  },
  projects: [
    {
      name: "chromium",
      use: { ...devices["Desktop Chrome"] },
    },
  ],
});
