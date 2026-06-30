// frontend_v2.md §12 L4 — MovePage e2e (mock backend + vite dev + chromium).
//
// 검증 invariant (사용자 push: "진짜 동작 검증"):
//   1. WS 연결 — connected badge green
//   2. URDF static fetch — /robot/so101_6dof/urdf/so101_6dof.urdf 200
//   3. Capability snapshot — J1 row 표시 (motor topology 응답 도착)
//   4. Motor.Stream.RAW_STATE — raw position 표시 (mock motor 20Hz publish)
//   5. JogJ button hold → mock motor 가 받아 raw position 변화
//
// 외부 의존 (실행 전 띄움):
//   - mock backend (port 8000)
//   - frontend vite (port 5174)

import { expect, test } from "@playwright/test";

const MOVE_PATH = "/robots/so101_6dof_0/move";

function readJ1Raw(): string {
  // RobotStatePanel 의 joint table — J1 row 의 3번째 셀 (raw)
  const rows = Array.from(
    document.querySelectorAll("div.grid.grid-cols-3"),
  );
  for (const row of rows) {
    const cells = row.children;
    if (cells.length === 3 && cells[0].textContent?.trim() === "J1") {
      return cells[2].textContent?.trim() ?? "";
    }
  }
  return "";
}

test.describe("MovePage e2e (mock backend)", () => {
  test("WS 연결 + URDF fetch + Motor.Stream.RAW_STATE 도착", async ({ page }) => {
    // URDF GET 캡처
    const urdfReq = page.waitForResponse(
      (res) =>
        res.url().includes("/robot/so101_6dof/urdf/so101_6dof.urdf") &&
        res.status() === 200,
      { timeout: 10_000 },
    );

    await page.goto(MOVE_PATH);

    // connected badge
    await expect(page.getByText("connected", { exact: true })).toBeVisible({
      timeout: 5_000,
    });

    // URDF 200
    await urdfReq;

    // J1 row 표시 (Motor.Service.GET_TOPOLOGY capability 도착)
    await expect(page.getByText("J1").first()).toBeVisible({ timeout: 5_000 });

    // Motor.Stream.RAW_STATE 의 positions_raw 도착 — raw cell !== "—"
    await page.waitForFunction(
      () => {
        const rows = Array.from(
          document.querySelectorAll("div.grid.grid-cols-3"),
        );
        for (const row of rows) {
          const cells = row.children;
          if (
            cells.length === 3 &&
            cells[0].textContent?.trim() === "J1" &&
            cells[2].textContent?.trim() !== "—"
          ) {
            return true;
          }
        }
        return false;
      },
      { timeout: 5_000 },
    );
  });

  // 진짜 사용자 jog cycle e2e — 사용자가 J1 + button 800ms hold 했을 때:
  //   1. JogJ panel 이 50Hz 로 jog_j publish
  //   2. backend Motion 이 받음 → SE(3) 적분 → Motor.Stream.COMMAND publish
  //   3. mock motor 가 받음 → state 변화 → Motor.Stream.RAW_STATE publish
  //   4. frontend RobotStatePanel 의 raw 가 변화 표시
  //
  // 핵심 fix: JogJ button 의 `setPointerCapture` — Chromium 이 button class 변경
  // 시 자동 pointercancel → pointerup promote 박는 자체 차단 (실 hardware 박은
  // 자리도 빠른 손가락 / 누른 채 드래그 시나리오 동일 fix).
  test("J1+ button 800ms hold → 50Hz publish + raw position 변화 (full wire e2e)", async ({
    page,
  }) => {
    page.on("console", (msg) => {
      const t = msg.text();
      if (t.includes("[JogJ]") || t.includes("[Bridge]")) {
        console.log("[browser]", t);
      }
    });

    await page.goto(MOVE_PATH);
    await expect(page.getByText("J1").first()).toBeVisible({ timeout: 5_000 });

    // 첫 RAW_STATE 도착
    await page.waitForFunction(
      () =>
        Array.from(document.querySelectorAll("div.grid.grid-cols-3")).some(
          (row) => {
            const cells = row.children;
            return (
              cells.length === 3 &&
              cells[0].textContent?.trim() === "J1" &&
              cells[2].textContent?.trim() !== "—"
            );
          },
        ),
      { timeout: 5_000 },
    );

    const initialRaw = await page.evaluate(readJ1Raw);

    // JogJ 의 J1+ button. RobotStatePanel 에는 "+" button 없음, tab button 은
    // "joint"/"tcp" — 첫 "+" = JogJ J1+.
    const jogPlus = page.locator('button:has-text("+")').first();
    await expect(jogPlus).toBeVisible();

    // CDP Input.dispatchTouchEvent 박은 자리 진짜 touch hold. Playwright Mouse.down
    // 박은 자리 chromium mouse cascade 100ms 시점 pointerup auto promote — 실제로
    // hold 박지 X. CDP touch event 박은 자리 사용자가 touchEnd 호출 박을 때까지 hold.
    const box = (await jogPlus.boundingBox())!;
    const x = box.x + box.width / 2;
    const y = box.y + box.height / 2;
    const cdp = await page.context().newCDPSession(page);
    await cdp.send("Input.dispatchTouchEvent", {
      type: "touchStart",
      touchPoints: [{ x, y, id: 1 }],
    });
    await new Promise<void>((r) => setTimeout(r, 800));
    await cdp.send("Input.dispatchTouchEvent", {
      type: "touchEnd",
      touchPoints: [],
    });

    // 800ms hold + backend cycle 시간 (Motion → motor cmd → state publish)
    await new Promise<void>((r) => setTimeout(r, 300));

    // raw 변화 검증 — 0.18 rad/s × 0.8s = 0.144 rad ≈ 8.25°, raw 변화 ~94 unit.
    // 25% allowance = 20 이상.
    const afterRaw = await page.evaluate(readJ1Raw);
    expect(afterRaw).not.toBe(initialRaw);
    expect(afterRaw).not.toBe("—");
    const initialNum = parseInt(initialRaw, 10);
    const afterNum = parseInt(afterRaw, 10);
    expect(Math.abs(afterNum - initialNum)).toBeGreaterThan(20);
  });
});
