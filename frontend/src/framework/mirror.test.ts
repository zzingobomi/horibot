// frontend.md §12.2 useMirror — 5 invariant.

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, renderHook, waitFor } from "@testing-library/react";
import { bridge } from "@/api/bridge";
import { useFrameworkStore } from "./store";
import { useMirror } from "./mirror";

const SNAPSHOT = "srv/calibration/snapshot_bundle" as const;
const CHANGE_TOPIC = "event/calibration/activated" as const;

beforeEach(() => {
  // useMirror 가 bridgeConnected dep — test 에선 connected=true 가정
  useFrameworkStore.setState({
    topicData: {},
    serviceData: {},
    bridgeConnected: true,
  });
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("useMirror", () => {
  // spec frontend.md §12.2 — invariant: mount 시 1회 snapshot fetch
  it("mount 시 snapshot fetch — service 1회 호출", async () => {
    const spy = vi.spyOn(bridge, "callService").mockResolvedValue({
      success: true,
      message: "",
      data: { bundle_id: 1 } as never,
    });

    const { result } = renderHook(() =>
      useMirror({
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        snapshotService: SNAPSHOT as any,
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        changeTopic: CHANGE_TOPIC as any,
      }),
    );

    await waitFor(() => expect(result.current.isReady).toBe(true));
    expect(spy).toHaveBeenCalledTimes(1);
    expect(result.current.value).toEqual({ bundle_id: 1 });
  });

  // spec frontend.md §12.2 — invariant: change event → snapshot 재호출 (payload 박지 X)
  it("change event 도착 → snapshot refetch (event payload 사용 안 함)", async () => {
    const spy = vi.spyOn(bridge, "callService")
      .mockResolvedValueOnce({ success: true, message: "", data: { bundle_id: 1 } as never })
      .mockResolvedValueOnce({ success: true, message: "", data: { bundle_id: 2 } as never });

    const { result } = renderHook(() =>
      useMirror({
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        snapshotService: SNAPSHOT as any,
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        changeTopic: CHANGE_TOPIC as any,
      }),
    );

    // 첫 snapshot 도착 후 isReady
    await waitFor(() => expect(result.current.value).toEqual({ bundle_id: 1 }));
    expect(spy).toHaveBeenCalledTimes(1);

    // change event publish — *payload 박지 X*, snapshot refetch trigger
    act(() => {
      useFrameworkStore
        .getState()
        .setTopicData(CHANGE_TOPIC, { robot_id: "x", bundle_id: 99 });
    });

    await waitFor(() => expect(result.current.value).toEqual({ bundle_id: 2 }));
    expect(spy).toHaveBeenCalledTimes(2);
    // event payload 의 bundle_id=99 박지 X — snapshot 의 bundle_id=2 박힘 (invalidate+refetch only)
  });

  // spec frontend.md §12.2 — invariant: snapshot 받은 후 isReady=true
  it("isReady — mount 직후 false → snapshot resolve 후 true", async () => {
    let resolveFn: ((value: never) => void) | null = null;
    vi.spyOn(bridge, "callService").mockImplementation(
      () => new Promise((resolve) => {
        resolveFn = resolve as never;
      }),
    );

    const { result } = renderHook(() =>
      useMirror({
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        snapshotService: SNAPSHOT as any,
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        changeTopic: CHANGE_TOPIC as any,
      }),
    );

    // resolve 박지 X 한 자리 — isReady=false
    await new Promise((r) => setTimeout(r, 30));
    expect(result.current.isReady).toBe(false);
    expect(result.current.value).toBeNull();

    // resolve 후 — isReady=true
    await act(async () => {
      resolveFn!({ success: true, message: "", data: { ok: true } } as never);
      await new Promise((r) => setTimeout(r, 10));
    });
    await waitFor(() => expect(result.current.isReady).toBe(true));
  });

  // spec frontend.md §12.2 — invariant: Owner 안 떠 있음 (snapshot fail) → cache=null 유지
  it("Owner 안 떠 있음 — service reject → isReady=false 유지", async () => {
    vi.spyOn(bridge, "callService").mockRejectedValue(new Error("Owner missing"));

    const { result } = renderHook(() =>
      useMirror({
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        snapshotService: SNAPSHOT as any,
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        changeTopic: CHANGE_TOPIC as any,
      }),
    );

    await new Promise((r) => setTimeout(r, 30));
    expect(result.current.isReady).toBe(false);
    expect(result.current.value).toBeNull();
  });

  // spec frontend.md §12.2 — invariant: unmount → cancelled flag → setState 박지 X
  it("unmount cleanup — unmount 후 응답 도착해도 setValue 안 함 (no React warning)", async () => {
    let resolveFn: ((value: never) => void) | null = null;
    vi.spyOn(bridge, "callService").mockImplementation(
      () => new Promise((resolve) => {
        resolveFn = resolve as never;
      }),
    );
    const errSpy = vi.spyOn(console, "error").mockImplementation(() => {});

    const { unmount } = renderHook(() =>
      useMirror({
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        snapshotService: SNAPSHOT as any,
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        changeTopic: CHANGE_TOPIC as any,
      }),
    );

    unmount();
    // unmount 후 resolve — setState 호출 안 됨. React warning 안 나옴.
    resolveFn!({ success: true, message: "", data: { ok: true } } as never);
    await new Promise((r) => setTimeout(r, 30));

    // React 의 "Can't perform a React state update on an unmounted component" warning 박지 X
    const reactWarnings = errSpy.mock.calls.filter((call) =>
      String(call[0]).includes("unmounted"),
    );
    expect(reactWarnings).toHaveLength(0);
  });
});
