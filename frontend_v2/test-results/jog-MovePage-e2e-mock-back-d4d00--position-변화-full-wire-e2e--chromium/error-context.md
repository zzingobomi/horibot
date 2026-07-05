# Instructions

- Following Playwright test failed.
- Explain why, be concise, respect Playwright best practices.
- Provide a snippet of code with the fix, if possible.

# Test info

- Name: jog.spec.ts >> MovePage e2e (mock backend) >> J1+ button 800ms hold → 50Hz publish + raw position 변화 (full wire e2e)
- Location: e2e\jog.spec.ts:75:3

# Error details

```
Error: expect(received).toBeGreaterThan(expected)

Expected: > 20
Received:   11
```

# Page snapshot

```yaml
- generic [ref=e3]:
  - complementary [ref=e4]:
    - generic [ref=e5]:
      - generic [ref=e6]:
        - heading "Horibot" [level=1] [ref=e7]
        - paragraph [ref=e8]: Robot Arm Controller
      - button "사이드바 접기" [ref=e9]:
        - img [ref=e10]
    - navigation [ref=e13]:
      - link "Tasks" [ref=e14] [cursor=pointer]:
        - /url: /tasks
        - img [ref=e15]
        - generic [ref=e18]: Tasks
      - paragraph [ref=e19]: Robots
      - generic [ref=e20]:
        - generic [ref=e21]:
          - img [ref=e22]
          - generic [ref=e25]: omx_f_0
        - link "Move" [ref=e26] [cursor=pointer]:
          - /url: /robots/omx_f_0/move
          - generic [ref=e27]: Move
        - link "Calibrate" [ref=e28] [cursor=pointer]:
          - /url: /robots/omx_f_0/calibrate
          - generic [ref=e29]: Calibrate
        - link "Assets" [ref=e30] [cursor=pointer]:
          - /url: /robots/omx_f_0/assets
          - generic [ref=e31]: Assets
      - generic [ref=e32]:
        - generic [ref=e33]:
          - img [ref=e34]
          - generic [ref=e37]: so101_6dof_0
        - link "Move" [ref=e38] [cursor=pointer]:
          - /url: /robots/so101_6dof_0/move
          - generic [ref=e39]: Move
        - link "Calibrate" [ref=e40] [cursor=pointer]:
          - /url: /robots/so101_6dof_0/calibrate
          - generic [ref=e41]: Calibrate
        - link "Scan" [ref=e42] [cursor=pointer]:
          - /url: /robots/so101_6dof_0/scan
          - generic [ref=e43]: Scan
        - link "Assets" [ref=e44] [cursor=pointer]:
          - /url: /robots/so101_6dof_0/assets
          - generic [ref=e45]: Assets
    - generic [ref=e46]:
      - paragraph [ref=e47]: Dev
      - link "Contract graph" [ref=e48] [cursor=pointer]:
        - /url: /contract
        - img [ref=e49]
        - generic [ref=e55]: Contract graph
    - generic [ref=e59]: online
  - main [ref=e60]:
    - generic [ref=e61]:
      - generic:
        - generic:
          - generic:
            - generic:
              - generic [ref=e75]:
                - generic [ref=e82] [cursor=pointer]: Robot State
                - generic [ref=e86]:
                  - generic [ref=e87]:
                    - generic [ref=e88]: connected
                    - generic [ref=e89]: torque off
                  - button "torque on" [ref=e90]
                  - generic [ref=e91]:
                    - generic [ref=e92]: joints (arm)
                    - generic [ref=e93]:
                      - generic [ref=e94]:
                        - generic [ref=e95]: name
                        - generic [ref=e96]: deg
                        - generic [ref=e97]: raw
                      - generic [ref=e98]:
                        - generic [ref=e99]: J1
                        - generic [ref=e100]: "5.8"
                        - generic [ref=e101]: "2114"
                      - generic [ref=e102]:
                        - generic [ref=e103]: J2
                        - generic [ref=e104]: "0.0"
                        - generic [ref=e105]: "2048"
                      - generic [ref=e106]:
                        - generic [ref=e107]: J3
                        - generic [ref=e108]: "-5.0"
                        - generic [ref=e109]: "1991"
                      - generic [ref=e110]:
                        - generic [ref=e111]: J4
                        - generic [ref=e112]: "0.0"
                        - generic [ref=e113]: "2048"
                      - generic [ref=e114]:
                        - generic [ref=e115]: J5
                        - generic [ref=e116]: "0.0"
                        - generic [ref=e117]: "2048"
                      - generic [ref=e118]:
                        - generic [ref=e119]: J6
                        - generic [ref=e120]: "0.0"
                        - generic [ref=e121]: "2048"
                  - generic [ref=e122]:
                    - generic [ref=e123]: tcp (m)
                    - generic [ref=e124]:
                      - generic [ref=e125]: x 0.262
                      - generic [ref=e126]: y -0.054
                      - generic [ref=e127]: z 0.179
              - generic [ref=e137]:
                - generic [ref=e144] [cursor=pointer]: Motion
                - generic [ref=e149]:
                  - tablist [ref=e150]:
                    - tab "joint" [selected] [ref=e151]
                    - tab "tcp" [ref=e152]
                  - tabpanel "joint" [ref=e153]:
                    - generic [ref=e154]:
                      - paragraph [ref=e155]: 버튼 hold = joint velocity publish (50Hz). backend latch + dt 적분 → URDF rad target. cross-process safe.
                      - generic [ref=e156]:
                        - generic [ref=e157]:
                          - generic [ref=e158]:
                            - generic [ref=e159]: J1
                            - generic [ref=e160]: 5.8°
                          - button "−" [ref=e161]
                          - button "+" [ref=e162]
                        - generic [ref=e163]:
                          - generic [ref=e164]:
                            - generic [ref=e165]: J2
                            - generic [ref=e166]: 0.0°
                          - button "−" [ref=e167]
                          - button "+" [ref=e168]
                        - generic [ref=e169]:
                          - generic [ref=e170]:
                            - generic [ref=e171]: J3
                            - generic [ref=e172]: "-5.0°"
                          - button "−" [ref=e173]
                          - button "+" [ref=e174]
                        - generic [ref=e175]:
                          - generic [ref=e176]:
                            - generic [ref=e177]: J4
                            - generic [ref=e178]: 0.0°
                          - button "−" [ref=e179]
                          - button "+" [ref=e180]
                        - generic [ref=e181]:
                          - generic [ref=e182]:
                            - generic [ref=e183]: J5
                            - generic [ref=e184]: 0.0°
                          - button "−" [ref=e185]
                          - button "+" [ref=e186]
                        - generic [ref=e187]:
                          - generic [ref=e188]:
                            - generic [ref=e189]: J6
                            - generic [ref=e190]: 0.0°
                          - button "−" [ref=e191]
                          - button "+" [ref=e192]
                      - generic [ref=e193]:
                        - generic [ref=e194]: 속도
                        - slider [ref=e199]
                        - generic [ref=e200]: 0.18 rad/s
      - button "Reset layout" [ref=e201]:
        - img [ref=e202]
        - text: Reset layout
      - generic:
        - generic:
          - generic: so101_6dof_0
          - generic: "type: so101_6dof"
```

# Test source

```ts
  15  | //
  16  | // 외부 의존 (실행 전 띄움):
  17  | //   - mock backend (port 8000): cd backend_v2 && uv run --no-sync python -m apps.main --host mock
  18  | //   - frontend vite (port 5174): cd frontend_v2 && pnpm dev
  19  | 
  20  | import { expect, test } from "@playwright/test";
  21  | 
  22  | const MOVE_PATH = "/robots/so101_6dof_0/move";
  23  | 
  24  | function readJ1Raw(): string {
  25  |   // RobotStatePanel 의 joint table — J1 row 의 3번째 셀 (raw)
  26  |   const rows = Array.from(document.querySelectorAll("div.grid.grid-cols-3"));
  27  |   for (const row of rows) {
  28  |     const cells = row.children;
  29  |     if (cells.length === 3 && cells[0].textContent?.trim() === "J1") {
  30  |       return cells[2].textContent?.trim() ?? "";
  31  |     }
  32  |   }
  33  |   return "";
  34  | }
  35  | 
  36  | function j1RawArrived(): boolean {
  37  |   return Array.from(document.querySelectorAll("div.grid.grid-cols-3")).some(
  38  |     (row) => {
  39  |       const c = row.children;
  40  |       return (
  41  |         c.length === 3 &&
  42  |         c[0].textContent?.trim() === "J1" &&
  43  |         c[2].textContent?.trim() !== "—"
  44  |       );
  45  |     },
  46  |   );
  47  | }
  48  | 
  49  | test.describe("MovePage e2e (mock backend)", () => {
  50  |   test("WS 연결 + URDF fetch + Motor.Stream.RAW_STATE 도착", async ({ page }) => {
  51  |     const urdfReq = page.waitForResponse(
  52  |       (res) =>
  53  |         res.url().includes("/robot/so101_6dof/urdf/so101_6dof.urdf") &&
  54  |         res.status() === 200,
  55  |       { timeout: 10_000 },
  56  |     );
  57  | 
  58  |     await page.goto(MOVE_PATH);
  59  | 
  60  |     await expect(page.getByText("online", { exact: true })).toBeVisible({
  61  |       timeout: 5_000,
  62  |     });
  63  |     await urdfReq;
  64  |     await expect(page.getByText("J1").first()).toBeVisible({ timeout: 5_000 });
  65  |     await page.waitForFunction(j1RawArrived, { timeout: 5_000 });
  66  |   });
  67  | 
  68  |   // 진짜 사용자 jog cycle — J1+ button 800ms hold 시:
  69  |   //   1. JogJ 가 50Hz 로 Motion.Stream.JOG_J publish
  70  |   //   2. backend Motion 이 SE(3) 적분 → Motor.Stream.COMMAND publish
  71  |   //   3. mock motor 가 받음 → write_positions → Motor.Stream.RAW_STATE 변화
  72  |   //   4. frontend RobotStatePanel 의 raw 가 변화 표시
  73  |   //
  74  |   // plain page.mouse — 실 사용자(마우스) 와 동일 입력. CDP touch / hasTouch 불필요.
  75  |   test("J1+ button 800ms hold → 50Hz publish + raw position 변화 (full wire e2e)", async ({
  76  |     page,
  77  |   }) => {
  78  |     // 진짜 upstream jog publish 수 카운트 (console 은 coalesce 되어 부정확) —
  79  |     // 50Hz 가 실제로 나가는지 검증.
  80  |     await page.addInitScript(() => {
  81  |       const w = window as unknown as { __jogSent: number };
  82  |       w.__jogSent = 0;
  83  |       const orig = WebSocket.prototype.send;
  84  |       WebSocket.prototype.send = function (
  85  |         data: string | ArrayBufferLike | Blob | ArrayBufferView,
  86  |       ) {
  87  |         if (typeof data === "string" && data.includes("jog_j")) w.__jogSent++;
  88  |         return orig.call(this, data as never);
  89  |       };
  90  |     });
  91  | 
  92  |     await page.goto(MOVE_PATH);
  93  |     await expect(page.getByText("J1").first()).toBeVisible({ timeout: 5_000 });
  94  |     await page.waitForFunction(j1RawArrived, { timeout: 5_000 });
  95  | 
  96  |     const initialRaw = await page.evaluate(readJ1Raw);
  97  | 
  98  |     // JogJ 의 J1+ button (RobotStatePanel 에는 "+" 없음, 첫 "+" = JogJ J1+).
  99  |     const jogPlus = page.locator('button:has-text("+")').first();
  100 |     await expect(jogPlus).toBeVisible();
  101 |     const box = (await jogPlus.boundingBox())!;
  102 |     const x = box.x + box.width / 2;
  103 |     const y = box.y + box.height / 2;
  104 | 
  105 |     await page.mouse.move(x, y);
  106 |     await page.mouse.down();
  107 |     await page.waitForTimeout(800);
  108 |     await page.mouse.up();
  109 |     await page.waitForTimeout(300); // backend cycle (Motion → motor cmd → raw publish)
  110 | 
  111 |     // 50Hz × 0.8s ≈ 40 publish. headed 실측 ~38-42. starve 회귀 차단 = 20 이상.
  112 |     const jogSent = await page.evaluate(
  113 |       () => (window as unknown as { __jogSent: number }).__jogSent,
  114 |     );
> 115 |     expect(jogSent).toBeGreaterThan(20);
      |                     ^ Error: expect(received).toBeGreaterThan(expected)
  116 | 
  117 |     // raw 변화 — 0.18 rad/s × 0.8s ≈ 0.144 rad ≈ raw 94 unit. allowance 25% = 20.
  118 |     const afterRaw = await page.evaluate(readJ1Raw);
  119 |     expect(afterRaw).not.toBe("—");
  120 |     expect(afterRaw).not.toBe(initialRaw);
  121 |     const delta = Math.abs(parseInt(afterRaw, 10) - parseInt(initialRaw, 10));
  122 |     expect(delta).toBeGreaterThan(20);
  123 |   });
  124 | });
  125 | 
```