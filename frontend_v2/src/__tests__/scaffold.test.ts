// frontend_v2.md §12.5 Step F1.5 — Vitest scaffold 동작 검증.
// 본 test 는 *meaningful invariant* 검증하지 않음 — Vitest 가 도는지 smoke check 만.
// Step F2 부터 invariant test 박힘.

import { describe, expect, it } from "vitest";

describe("Vitest scaffold", () => {
  it("Vitest 동작 + @ alias import OK", async () => {
    const constants = await import("@/constants");
    expect(constants.DEFAULT_ROBOT_ID).toBe("so101_6dof_0");
    expect(constants.WS_URL).toMatch(/^wss?:\/\//);
  });
});
