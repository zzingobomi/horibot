// gen-contract.mjs render 검증 — frontend_contract_gen.md §6 (frontend CONSUME 쪽).
//
// 경계: 이 테스트는 backend 를 일절 참조하지 않는다. render 는 순수 함수라 fixture
// (한 번 캡처한 /contract.json 샘플) 만으로 검증. backend↔frontend end-to-end 정합
// (backend JSON → contract.ts) 는 verify 단계(mock 띄우고 `pnpm gen:types` → diff)가
// 담당 — 런타임/테스트 커플링 아님.
//
// `?raw` = vite 가 파일 내용을 문자열로 import (vite/client 타입). node:fs 불필요.

import { describe, expect, it } from "vitest";

// @ts-expect-error — .mjs 빌드 도구 (타입 선언 없음, render 는 pure JS)
import { renderContractTs } from "../../scripts/gen-contract.mjs";
import contractTs from "./generated/contract.ts?raw";
import fixtureRaw from "./__fixtures__/contract.json?raw";

const lf = (s: string) => s.replace(/\r\n/g, "\n");

describe("gen-contract render", () => {
  it("fixture → 커밋된 contract.ts 재생성 (byte-identical, regen invariant)", () => {
    const fixture = JSON.parse(fixtureRaw);
    expect(lf(renderContractTs(fixture))).toBe(lf(contractTs));
  });

  it("enum / interface / topic / service 구조 조립", () => {
    const ts: string = renderContractTs({
      enums: [{ name: "Foo", members: [["A", "a"]] }],
      interfaces: [
        { name: "Bar", fields: [{ name: "x", ts: "number", optional: false }] },
      ],
      topics: [{ const: "T_X", key: "stream/x", payload: "Bar" }],
      services: [{ const: "S_Y", key: "srv/y", req: "Bar", res: "Bar" }],
    });
    expect(ts).toContain("export const Foo = {");
    expect(ts).toContain("  A: \"a\",");
    expect(ts).toContain("export interface Bar {");
    expect(ts).toContain("  x: number;");
    expect(ts).toContain("  T_X: \"stream/x\",");
    expect(ts).toContain("\"stream/x\": Bar;");
    expect(ts).toContain("\"srv/y\": { req: Bar; res: Bar };");
  });

  it("draft 계약은 @draft JSDoc + DRAFT_CONTRACTS set (draft 있을 때만)", () => {
    const withDraft: string = renderContractTs({
      enums: [],
      interfaces: [{ name: "Loose", fields: [], draft: true }],
      topics: [{ const: "T_X", key: "stream/x", payload: "Loose", draft: true }],
      services: [
        { const: "S_Y", key: "srv/y", req: "Loose", res: "Loose", draft: false },
      ],
    });
    expect(withDraft).toContain("/** @draft");
    expect(withDraft).toContain("export const DRAFT_CONTRACTS: ReadonlySet<string>");
    // draft set 은 draft=true wire 만 — draft topic 포함, draft=false service 제외
    const block = withDraft.slice(withDraft.indexOf("DRAFT_CONTRACTS"));
    expect(block).toContain('"stream/x",');
    expect(block).not.toContain('"srv/y"');
  });

  it("draft 없으면 @draft/DRAFT_CONTRACTS 미emit (재생성 불변)", () => {
    const noDraft: string = renderContractTs({
      enums: [],
      interfaces: [{ name: "Solid", fields: [], draft: false }],
      topics: [{ const: "T", key: "stream/z", payload: "Solid", draft: false }],
      services: [],
    });
    expect(noDraft).not.toContain("@draft");
    expect(noDraft).not.toContain("DRAFT_CONTRACTS");
  });

  it("optional 필드는 ?: 로 emit", () => {
    const ts: string = renderContractTs({
      enums: [],
      interfaces: [
        {
          name: "Opt",
          fields: [
            { name: "req", ts: "string", optional: false },
            { name: "maybe", ts: "number | null", optional: true },
          ],
        },
      ],
      topics: [],
      services: [],
    });
    expect(ts).toContain("  req: string;");
    expect(ts).toContain("  maybe?: number | null;");
  });
});
