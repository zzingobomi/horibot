// frontend_v2.md §12 L4 — WebdriverIO PoC.
// W3C Actions API 의 pointerDown + pause + pointerUp — real wall-clock hold.
// Playwright Mouse / CDP touch 박은 자리 100ms 시점 pointerup auto fire 박는
// 현상 회피 (root cause 박은 자리 confirmed 박지 X 단 현상 확실).

import { browser, $$ } from "@wdio/globals";

const MOVE_PATH = "/robots/so101_6dof_0/move";
const HOLD_MS = 800;

async function readJ1Raw(): Promise<string> {
  return browser.execute(() => {
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
  });
}

describe("MovePage e2e (WebdriverIO)", () => {
  it(`J1+ button ${HOLD_MS}ms hold → 50Hz publish + raw position 변화`, async () => {
    await browser.url(MOVE_PATH);

    // J1 row + first RAW_STATE 도착 wait. textContent (chromedriver innerText
    // viewport-visible-only 박은 자리 회피).
    await browser.waitUntil(
      async () =>
        await browser.execute(() => {
          const rows = Array.from(
            document.querySelectorAll("div.grid.grid-cols-3"),
          );
          return rows.some((row) => {
            const cells = row.children;
            return (
              cells.length === 3 &&
              cells[0].textContent?.trim() === "J1" &&
              cells[2].textContent?.trim() !== "—" &&
              cells[2].textContent?.trim() !== ""
            );
          });
        }),
      { timeout: 10_000, timeoutMsg: "J1 raw position 도착 안 함" },
    );

    const initialRaw = await readJ1Raw();
    expect(initialRaw).not.toEqual("");
    expect(initialRaw).not.toEqual("—");

    // JogJ 의 J1+ button — buttonTexts ["torque on","joint","tcp","−","+",...]
    // 첫 "+" 박은 자리 4번째 button (index 4 = J1 +).
    const buttons = await $$("button");
    let jogPlus: WebdriverIO.Element | null = null;
    for (const btn of buttons) {
      const text = (await btn.getText()) || "";
      if (text === "+") {
        jogPlus = btn;
        break;
      }
    }
    if (!jogPlus) throw new Error("JogJ J1+ button 박지 X");

    // W3C Actions API — move + pointerDown + pause(800ms real wall-clock) +
    // pointerUp. WebDriver chromedriver 박은 자리 raw pointer event 박음.
    const rect = await jogPlus.getElementRect(jogPlus.elementId);
    const x = Math.round(rect.x + rect.width / 2);
    const y = Math.round(rect.y + rect.height / 2);

    await browser
      .action("pointer", { parameters: { pointerType: "mouse" } })
      .move({ x, y })
      .down({ button: 0 })
      .pause(HOLD_MS)
      .up({ button: 0 })
      .perform();

    // backend cycle 처리 시간 (Motion → motor cmd → mock motor → raw publish)
    await browser.pause(300);

    // debug — JogJ tick console log 캡처
    try {
      const logs = await browser.getLogs("browser");
      const jogLogs = (logs as Array<{ message: string }>)
        .filter((l) => l.message.includes("JogJ"))
        .slice(-15);
      console.log("[wdio] JogJ logs (last 15):", JSON.stringify(jogLogs, null, 2));
    } catch (e) {
      console.log("[wdio] getLogs:", String(e));
    }

    const afterRaw = await readJ1Raw();
    expect(afterRaw).not.toEqual("—");
    expect(afterRaw).not.toEqual(initialRaw);

    // 변화 크기 — 0.18 rad/s × 0.8s = 0.144 rad ≈ 8.25° → raw ≈ 94 unit.
    // allowance 25% = 20.
    const initialNum = parseInt(initialRaw, 10);
    const afterNum = parseInt(afterRaw, 10);
    expect(Math.abs(afterNum - initialNum)).toBeGreaterThan(20);
  });
});
