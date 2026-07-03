// contract_graph_viewer.md §7-7 L4 — /contract 페이지 e2e (mock backend + vite dev).
//
// 검증 invariant:
//   1. GET /contract/graph 200 (unfiltered 계약 그래프 export)
//   2. 9 contentful module 노드 렌더 (declared universe — bridge 제외.
//      test_contract_export 의 registry 집합과 동일 SSOT)
//   3. 방향 엣지 렌더 (React Flow edge)
//   4. legend 의 module/edge 카운트
//   5. 엣지 클릭 → payload 스키마 드릴다운 패널
//
// 외부 의존 (실행 전 띄움):
//   - mock backend (port 8000): cd backend_v2 && uv run --no-sync python -m apps.main --host mock
//   - frontend vite (port 5174): cd frontend_v2 && pnpm dev

import { expect, test } from "@playwright/test";

// declared universe (MODULE_REGISTRY contentful) — backend test_contract_export 정합
const MODULE_IDS = [
  "MotorDriverModule",
  "MotionModule",
  "CameraDriverModule",
  "CameraDecodedModule",
  "CalibrationModule",
  "Scene3DModule",
  "ScanModule",
  "WaypointModule",
  "DetectorModule",
];

test.describe("Contract graph viewer e2e (mock backend)", () => {
  test("/contract 로드 → GET /contract/graph 200 + module 노드 + 엣지 렌더", async ({
    page,
  }) => {
    const graphReq = page.waitForResponse(
      (res) =>
        res.url().includes("/contract/graph") && res.status() === 200,
      { timeout: 10_000 },
    );

    await page.goto("/contract");
    await graphReq;

    // contentful module 노드 전부 — bridge (contract 0) 는 제외
    for (const id of MODULE_IDS) {
      await expect(page.getByText(id, { exact: true })).toBeVisible({
        timeout: 5_000,
      });
    }
    await expect(page.getByText("BridgeModule")).toHaveCount(0);

    // React Flow 노드/엣지 실제 DOM 렌더
    await expect(page.locator(".react-flow__node")).toHaveCount(
      MODULE_IDS.length,
    );
    const edgeCount = await page.locator(".react-flow__edge").count();
    expect(edgeCount).toBeGreaterThanOrEqual(4);

    // legend 카운트 (main 영역 — sidebar 의 "Contract graph" 링크와 구분)
    await expect(
      page.getByRole("main").getByText("Contract graph"),
    ).toBeVisible();
    await expect(
      page.getByText(new RegExp(`modules ${MODULE_IDS.length} · edges`)),
    ).toBeVisible();
  });

  test("엣지 클릭 → payload 스키마 드릴다운 패널", async ({ page }) => {
    await page.goto("/contract");
    await page.waitForResponse(
      (res) => res.url().includes("/contract/graph") && res.status() === 200,
      { timeout: 10_000 },
    );
    await expect(page.locator(".react-flow__node")).toHaveCount(
      MODULE_IDS.length,
    );

    // 첫 엣지 클릭 (interaction path — 넓은 hit area). onEdgeClick → SchemaPanel.
    const edge = page.locator(".react-flow__edge").first();
    await edge.click({ force: true });

    // 패널에 payload model 스키마 (field: ts_type) 표시 — "payload" 라벨 + 닫기 버튼
    await expect(page.getByText("payload", { exact: true })).toBeVisible({
      timeout: 5_000,
    });
    await expect(page.getByRole("button", { name: "✕" })).toBeVisible();
  });
});
