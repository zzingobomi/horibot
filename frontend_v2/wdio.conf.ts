import type { Options } from "@wdio/types";

// frontend_v2.md §12 L4 — WebdriverIO + W3C Actions API.
// Playwright 박은 자리 pointerdown hold 박은 자리 chromium native event 박은 자리
// pointerup ~100ms 시점 fire (root cause confirmed 박지 X — 단 현상 확실).
// WebdriverIO 박은 자리 W3C WebDriver Actions API 의 pointerDown + pause + pointerUp
// 박은 자리 real wall-clock 박은 자리 — chromium driver 박은 자리 raw pointer 박음.
//
// 외부 의존 (실행 전 띄움):
//   - mock backend (port 8000)
//   - frontend vite (port 5174)

export const config: Options.Testrunner = {
  runner: "local",
  framework: "mocha",
  specs: ["./e2e_wdio/**/*.test.ts"],
  maxInstances: 1,
  capabilities: [
    {
      browserName: "chrome",
      "goog:chromeOptions": {
        args: [
          "--disable-search-engine-choice-screen",
          "--window-size=1600,1000", // MovePage 의 3-column grid 가 viewport 안 들어가게
        ],
      },
    },
  ],
  logLevel: "warn",
  baseUrl: "http://localhost:5174",
  waitforTimeout: 5_000,
  connectionRetryTimeout: 120_000,
  connectionRetryCount: 3,
  reporters: ["spec"],
  mochaOpts: {
    ui: "bdd",
    timeout: 60_000,
  },
  autoCompileOpts: {
    autoCompile: true,
    tsNodeOpts: {
      transpileOnly: true,
      project: "./tsconfig.wdio.json",
    },
  },
};
