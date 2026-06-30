# Instructions

- Following Playwright test failed.
- Explain why, be concise, respect Playwright best practices.
- Provide a snippet of code with the fix, if possible.

# Test info

- Name: jog.spec.ts >> MovePage e2e (mock backend) >> J1+ button 800ms hold → 50Hz publish + raw position 변화 (full wire e2e)
- Location: e2e\jog.spec.ts:86:3

# Error details

```
Error: expect(received).not.toBe(expected) // Object.is equality

Expected: not "2099"
```

# Page snapshot

```yaml
- generic [ref=e3]:
  - generic [ref=e4]:
    - generic [ref=e5]: so101_6dof_0
    - generic [ref=e6]:
      - generic [ref=e7]:
        - generic [ref=e8]: connected
        - generic [ref=e9]: torque ?
        - generic [ref=e10]: stale 3734ms
      - button "torque on" [disabled] [ref=e11]
      - generic [ref=e12]:
        - generic [ref=e13]: joints (arm)
        - generic [ref=e14]:
          - generic [ref=e15]:
            - generic [ref=e16]: name
            - generic [ref=e17]: deg
            - generic [ref=e18]: raw
          - generic [ref=e19]:
            - generic [ref=e20]: J1
            - generic [ref=e21]: "4.5"
            - generic [ref=e22]: "2099"
          - generic [ref=e23]:
            - generic [ref=e24]: J2
            - generic [ref=e25]: "0.0"
            - generic [ref=e26]: "2048"
          - generic [ref=e27]:
            - generic [ref=e28]: J3
            - generic [ref=e29]: "-5.0"
            - generic [ref=e30]: "1991"
          - generic [ref=e31]:
            - generic [ref=e32]: J4
            - generic [ref=e33]: "0.0"
            - generic [ref=e34]: "2048"
          - generic [ref=e35]:
            - generic [ref=e36]: J5
            - generic [ref=e37]: "0.0"
            - generic [ref=e38]: "2048"
          - generic [ref=e39]:
            - generic [ref=e40]: J6
            - generic [ref=e41]: "0.0"
            - generic [ref=e42]: "2048"
      - generic [ref=e43]:
        - generic [ref=e44]: tcp (m)
        - generic [ref=e45]:
          - generic [ref=e46]: x 0.262
          - generic [ref=e47]: y -0.048
          - generic [ref=e48]: z 0.179
  - generic [ref=e53]:
    - generic [ref=e54]:
      - button "joint" [ref=e55]
      - button "tcp" [ref=e56]
    - generic [ref=e57]:
      - paragraph [ref=e58]: 버튼 hold = joint velocity publish (50Hz). backend latch + dt 적분 → URDF rad target. cross-process safe.
      - generic [ref=e59]:
        - generic [ref=e60]:
          - generic [ref=e61]:
            - generic [ref=e62]: J1
            - generic [ref=e63]: 4.5°
          - button "−" [ref=e64]
          - button "+" [ref=e65]
        - generic [ref=e66]:
          - generic [ref=e67]:
            - generic [ref=e68]: J2
            - generic [ref=e69]: 0.0°
          - button "−" [ref=e70]
          - button "+" [ref=e71]
        - generic [ref=e72]:
          - generic [ref=e73]:
            - generic [ref=e74]: J3
            - generic [ref=e75]: "-5.0°"
          - button "−" [ref=e76]
          - button "+" [ref=e77]
        - generic [ref=e78]:
          - generic [ref=e79]:
            - generic [ref=e80]: J4
            - generic [ref=e81]: 0.0°
          - button "−" [ref=e82]
          - button "+" [ref=e83]
        - generic [ref=e84]:
          - generic [ref=e85]:
            - generic [ref=e86]: J5
            - generic [ref=e87]: 0.0°
          - button "−" [ref=e88]
          - button "+" [ref=e89]
        - generic [ref=e90]:
          - generic [ref=e91]:
            - generic [ref=e92]: J6
            - generic [ref=e93]: 0.0°
          - button "−" [ref=e94]
          - button "+" [ref=e95]
      - generic [ref=e96]:
        - generic [ref=e97]: 속도
        - slider [ref=e98]: "0.3"
        - generic [ref=e99]: 0.18 rad/s
```

# Test source

```ts
  45  |     await expect(page.getByText("connected", { exact: true })).toBeVisible({
  46  |       timeout: 5_000,
  47  |     });
  48  | 
  49  |     // URDF 200
  50  |     await urdfReq;
  51  | 
  52  |     // J1 row 표시 (Motor.Service.GET_TOPOLOGY capability 도착)
  53  |     await expect(page.getByText("J1").first()).toBeVisible({ timeout: 5_000 });
  54  | 
  55  |     // Motor.Stream.RAW_STATE 의 positions_raw 도착 — raw cell !== "—"
  56  |     await page.waitForFunction(
  57  |       () => {
  58  |         const rows = Array.from(
  59  |           document.querySelectorAll("div.grid.grid-cols-3"),
  60  |         );
  61  |         for (const row of rows) {
  62  |           const cells = row.children;
  63  |           if (
  64  |             cells.length === 3 &&
  65  |             cells[0].textContent?.trim() === "J1" &&
  66  |             cells[2].textContent?.trim() !== "—"
  67  |           ) {
  68  |             return true;
  69  |           }
  70  |         }
  71  |         return false;
  72  |       },
  73  |       { timeout: 5_000 },
  74  |     );
  75  |   });
  76  | 
  77  |   // 진짜 사용자 jog cycle e2e — 사용자가 J1 + button 800ms hold 했을 때:
  78  |   //   1. JogJ panel 이 50Hz 로 jog_j publish
  79  |   //   2. backend Motion 이 받음 → SE(3) 적분 → Motor.Stream.COMMAND publish
  80  |   //   3. mock motor 가 받음 → state 변화 → Motor.Stream.RAW_STATE publish
  81  |   //   4. frontend RobotStatePanel 의 raw 가 변화 표시
  82  |   //
  83  |   // 핵심 fix: JogJ button 의 `setPointerCapture` — Chromium 이 button class 변경
  84  |   // 시 자동 pointercancel → pointerup promote 박는 자체 차단 (실 hardware 박은
  85  |   // 자리도 빠른 손가락 / 누른 채 드래그 시나리오 동일 fix).
  86  |   test("J1+ button 800ms hold → 50Hz publish + raw position 변화 (full wire e2e)", async ({
  87  |     page,
  88  |   }) => {
  89  |     page.on("console", (msg) => {
  90  |       const t = msg.text();
  91  |       if (t.includes("[JogJ]") || t.includes("[Bridge]")) {
  92  |         console.log("[browser]", t);
  93  |       }
  94  |     });
  95  | 
  96  |     await page.goto(MOVE_PATH);
  97  |     await expect(page.getByText("J1").first()).toBeVisible({ timeout: 5_000 });
  98  | 
  99  |     // 첫 RAW_STATE 도착
  100 |     await page.waitForFunction(
  101 |       () =>
  102 |         Array.from(document.querySelectorAll("div.grid.grid-cols-3")).some(
  103 |           (row) => {
  104 |             const cells = row.children;
  105 |             return (
  106 |               cells.length === 3 &&
  107 |               cells[0].textContent?.trim() === "J1" &&
  108 |               cells[2].textContent?.trim() !== "—"
  109 |             );
  110 |           },
  111 |         ),
  112 |       { timeout: 5_000 },
  113 |     );
  114 | 
  115 |     const initialRaw = await page.evaluate(readJ1Raw);
  116 | 
  117 |     // JogJ 의 J1+ button. RobotStatePanel 에는 "+" button 없음, tab button 은
  118 |     // "joint"/"tcp" — 첫 "+" = JogJ J1+.
  119 |     const jogPlus = page.locator('button:has-text("+")').first();
  120 |     await expect(jogPlus).toBeVisible();
  121 | 
  122 |     // CDP Input.dispatchTouchEvent 박은 자리 진짜 touch hold. Playwright Mouse.down
  123 |     // 박은 자리 chromium mouse cascade 100ms 시점 pointerup auto promote — 실제로
  124 |     // hold 박지 X. CDP touch event 박은 자리 사용자가 touchEnd 호출 박을 때까지 hold.
  125 |     const box = (await jogPlus.boundingBox())!;
  126 |     const x = box.x + box.width / 2;
  127 |     const y = box.y + box.height / 2;
  128 |     const cdp = await page.context().newCDPSession(page);
  129 |     await cdp.send("Input.dispatchTouchEvent", {
  130 |       type: "touchStart",
  131 |       touchPoints: [{ x, y, id: 1 }],
  132 |     });
  133 |     await new Promise<void>((r) => setTimeout(r, 800));
  134 |     await cdp.send("Input.dispatchTouchEvent", {
  135 |       type: "touchEnd",
  136 |       touchPoints: [],
  137 |     });
  138 | 
  139 |     // 800ms hold + backend cycle 시간 (Motion → motor cmd → state publish)
  140 |     await new Promise<void>((r) => setTimeout(r, 300));
  141 | 
  142 |     // raw 변화 검증 — 0.18 rad/s × 0.8s = 0.144 rad ≈ 8.25°, raw 변화 ~94 unit.
  143 |     // 25% allowance = 20 이상.
  144 |     const afterRaw = await page.evaluate(readJ1Raw);
> 145 |     expect(afterRaw).not.toBe(initialRaw);
      |                          ^ Error: expect(received).not.toBe(expected) // Object.is equality
  146 |     expect(afterRaw).not.toBe("—");
  147 |     const initialNum = parseInt(initialRaw, 10);
  148 |     const afterNum = parseInt(afterRaw, 10);
  149 |     expect(Math.abs(afterNum - initialNum)).toBeGreaterThan(20);
  150 |   });
  151 | });
  152 | 
```