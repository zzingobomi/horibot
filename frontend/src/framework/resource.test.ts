// frontend.md §12.2 useResource — 2 invariant.

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { renderHook, waitFor } from "@testing-library/react";
import { _resetResourceCache, useResource } from "./resource";

beforeEach(() => {
  _resetResourceCache();
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("useResource", () => {
  // spec frontend.md §12.2 — invariant: 같은 path 두 hook 박혀도 fetch 1회만
  it("module cache — 같은 path 두 hook 박혀도 fetch 1회만 호출", async () => {
    const fetchSpy = vi.spyOn(globalThis, "fetch").mockResolvedValue(
      new Response(JSON.stringify({ robots: [], default: null }), {
        status: 200,
        headers: { "Content-Type": "application/json" },
      }),
    );

    const { result: a } = renderHook(() => useResource("/robots"));
    const { result: b } = renderHook(() => useResource("/robots"));

    await waitFor(() => {
      expect(a.current.data).not.toBeNull();
      expect(b.current.data).not.toBeNull();
    });

    expect(fetchSpy).toHaveBeenCalledTimes(1);
  });

  // spec frontend.md §12.2 — invariant: refetch() 호출 시 force fetch
  it("refetch() — cache 박혀있어도 force fetch", async () => {
    const fetchSpy = vi.spyOn(globalThis, "fetch").mockResolvedValue(
      new Response(JSON.stringify({ value: 1 }), {
        status: 200,
        headers: { "Content-Type": "application/json" },
      }),
    );

    const { result } = renderHook(() => useResource<{ value: number }>("/v"));

    await waitFor(() => expect(result.current.data).toEqual({ value: 1 }));
    expect(fetchSpy).toHaveBeenCalledTimes(1);

    await result.current.refetch();
    expect(fetchSpy).toHaveBeenCalledTimes(2);
  });
});
