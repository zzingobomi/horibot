// frontend_v2.md §12 L4 — MovePage e2e (mock backend + vite dev + chromium headed).
//
// 검증 invariant:
//   1. WS 연결 — connected badge green
//   2. URDF static fetch — /robot/so101_6dof/urdf/so101_6dof.urdf 200
//   3. Capability snapshot — J1 row 표시 (motor topology 응답 도착)
//   4. Motor.Stream.RAW_STATE — raw position 표시 (mock motor 20Hz publish)
//   5. JogJ button hold → 50Hz publish → backend Motion 적분 → mock motor raw 변화
//
// 실행 환경 (2026-07-01 진단): headless 기본 SwiftShader(소프트웨어 렌더)는 R3F
// 3D 씬 렌더로 메인스레드를 굶겨 50Hz setInterval 이 ~10Hz 로 밀림 → jog publish
// rate 가 backend IDLE_RESET(0.2s) 아래로 떨어져 모터가 안 움직인다. playwright.
// config.ts `headless: false` (실 GPU) 로 메인스레드 자유 → 50Hz 회복. plain mouse
// hold 는 headless/headed 모두 pointercancel 없이 정상 — 입력 도구 문제가 아님.
//
// 외부 의존 (실행 전 띄움):
//   - mock backend (port 8000): cd backend_v2 && uv run --no-sync python -m apps.main --host mock
//   - frontend vite (port 5174): cd frontend_v2 && pnpm dev

import { expect, test } from "@playwright/test";

const MOVE_PATH = "/robots/so101_6dof_0/move";

function readJ1Raw(): string {
  // RobotStatePanel 의 joint table — J1 row 의 3번째 셀 (raw)
  const rows = Array.from(document.querySelectorAll("div.grid.grid-cols-3"));
  for (const row of rows) {
    const cells = row.children;
    if (cells.length === 3 && cells[0].textContent?.trim() === "J1") {
      return cells[2].textContent?.trim() ?? "";
    }
  }
  return "";
}

function j1RawArrived(): boolean {
  return Array.from(document.querySelectorAll("div.grid.grid-cols-3")).some(
    (row) => {
      const c = row.children;
      return (
        c.length === 3 &&
        c[0].textContent?.trim() === "J1" &&
        c[2].textContent?.trim() !== "—"
      );
    },
  );
}

test.describe("MovePage e2e (mock backend)", () => {
  test("WS 연결 + URDF fetch + Motor.Stream.RAW_STATE 도착", async ({ page }) => {
    const urdfReq = page.waitForResponse(
      (res) =>
        res.url().includes("/robot/so101_6dof/urdf/so101_6dof.urdf") &&
        res.status() === 200,
      { timeout: 10_000 },
    );

    await page.goto(MOVE_PATH);

    await expect(page.getByText("connected", { exact: true })).toBeVisible({
      timeout: 5_000,
    });
    await urdfReq;
    await expect(page.getByText("J1").first()).toBeVisible({ timeout: 5_000 });
    await page.waitForFunction(j1RawArrived, { timeout: 5_000 });
  });

  // 진짜 사용자 jog cycle — J1+ button 800ms hold 시:
  //   1. JogJ 가 50Hz 로 Motion.Stream.JOG_J publish
  //   2. backend Motion 이 SE(3) 적분 → Motor.Stream.COMMAND publish
  //   3. mock motor 가 받음 → write_positions → Motor.Stream.RAW_STATE 변화
  //   4. frontend RobotStatePanel 의 raw 가 변화 표시
  //
  // plain page.mouse — 실 사용자(마우스) 와 동일 입력. CDP touch / hasTouch 불필요.
  test("J1+ button 800ms hold → 50Hz publish + raw position 변화 (full wire e2e)", async ({
    page,
  }) => {
    // 진짜 upstream jog publish 수 카운트 (console 은 coalesce 되어 부정확) —
    // 50Hz 가 실제로 나가는지 검증.
    await page.addInitScript(() => {
      const w = window as unknown as { __jogSent: number };
      w.__jogSent = 0;
      const orig = WebSocket.prototype.send;
      WebSocket.prototype.send = function (
        data: string | ArrayBufferLike | Blob | ArrayBufferView,
      ) {
        if (typeof data === "string" && data.includes("jog_j")) w.__jogSent++;
        return orig.call(this, data as never);
      };
    });

    await page.goto(MOVE_PATH);
    await expect(page.getByText("J1").first()).toBeVisible({ timeout: 5_000 });
    await page.waitForFunction(j1RawArrived, { timeout: 5_000 });

    const initialRaw = await page.evaluate(readJ1Raw);

    // JogJ 의 J1+ button (RobotStatePanel 에는 "+" 없음, 첫 "+" = JogJ J1+).
    const jogPlus = page.locator('button:has-text("+")').first();
    await expect(jogPlus).toBeVisible();
    const box = (await jogPlus.boundingBox())!;
    const x = box.x + box.width / 2;
    const y = box.y + box.height / 2;

    await page.mouse.move(x, y);
    await page.mouse.down();
    await page.waitForTimeout(800);
    await page.mouse.up();
    await page.waitForTimeout(300); // backend cycle (Motion → motor cmd → raw publish)

    // 50Hz × 0.8s ≈ 40 publish. headed 실측 ~38-42. starve 회귀 차단 = 20 이상.
    const jogSent = await page.evaluate(
      () => (window as unknown as { __jogSent: number }).__jogSent,
    );
    expect(jogSent).toBeGreaterThan(20);

    // raw 변화 — 0.18 rad/s × 0.8s ≈ 0.144 rad ≈ raw 94 unit. allowance 25% = 20.
    const afterRaw = await page.evaluate(readJ1Raw);
    expect(afterRaw).not.toBe("—");
    expect(afterRaw).not.toBe(initialRaw);
    const delta = Math.abs(parseInt(afterRaw, 10) - parseInt(initialRaw, 10));
    expect(delta).toBeGreaterThan(20);
  });
});
