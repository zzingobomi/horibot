// frontend_v2.md §12.2 useService — 2 invariant.

import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act, renderHook } from "@testing-library/react";
import { encode as msgpackEncode } from "@msgpack/msgpack";
import { Server } from "mock-socket";
import { WS_URL } from "@/constants";
import { FRAME_VERSION, FrameType, WsOp } from "@/types/bridge";
import { bridge } from "@/api/bridge";
import { useFrameworkStore } from "./store";
import { useService } from "./service";

function encodeFrame(type: number, key: string, payload: Uint8Array): ArrayBuffer {
  const keyBytes = new TextEncoder().encode(key);
  const buf = new ArrayBuffer(4 + keyBytes.length + payload.length);
  const view = new DataView(buf);
  view.setUint8(0, FRAME_VERSION);
  view.setUint8(1, type);
  view.setUint16(2, keyBytes.length, false);
  new Uint8Array(buf, 4, keyBytes.length).set(keyBytes);
  new Uint8Array(buf, 4 + keyBytes.length, payload.length).set(payload);
  return buf;
}

let server: Server;

beforeEach(() => {
  useFrameworkStore.setState({
    topicData: {},
    serviceData: {},
    bridgeConnected: false,
  });
  server = new Server(WS_URL);
});

afterEach(() => {
  bridge.disconnect();
  server.stop();
});

function waitOpen(): Promise<void> {
  return new Promise((resolve) => {
    bridge.connect((connected) => {
      if (connected) resolve();
    });
  });
}

describe("useService", () => {
  // spec frontend_v2.md §12.2 — invariant: call 후 cache reactive 갱신 (data / success)
  it("call 후 cache 갱신 + reactive view (data + success) 검증", async () => {
    server.on("connection", (socket) => {
      socket.on("message", (raw) => {
        const msg = JSON.parse(raw as string);
        if (msg.op === WsOp.Service) {
          const env = msgpackEncode({
            timestamp: 1.0,
            data: { accepted: true, message: "ok" },
          });
          socket.send(encodeFrame(FrameType.ServiceResponse, msg.request_id, env));
        }
      });
    });
    await waitOpen();

    // any cast — test 가 임의 string key 사용 (generated key 가리키지 않음)
    const { result } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useService("srv/test/echo" as any),
    );

    expect(result.current.success).toBe(false);
    expect(result.current.data).toBeNull();

    await act(async () => {
      await result.current.call({} as never);
    });

    expect(result.current.success).toBe(true);
    expect(result.current.data).toEqual({ accepted: true, message: "ok" });
  });

  // spec frontend_v2.md §12.2 — invariant: pending = call 직후 true → response 후 false
  it("pending flag — call 직후 true → response 후 false", async () => {
    let respond: (() => void) | null = null;
    server.on("connection", (socket) => {
      socket.on("message", (raw) => {
        const msg = JSON.parse(raw as string);
        if (msg.op === WsOp.Service) {
          // 응답 지연 — pending 박힌 자리 검증용
          respond = () => {
            const env = msgpackEncode({ timestamp: 0, data: { ok: true } });
            socket.send(encodeFrame(FrameType.ServiceResponse, msg.request_id, env));
          };
        }
      });
    });
    await waitOpen();

    const { result } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useService("srv/test/delayed" as any),
    );

    let callPromise!: Promise<unknown>;
    act(() => {
      callPromise = result.current.call({} as never);
    });

    // 응답 도착 전 — pending=true 자리
    await new Promise((r) => setTimeout(r, 20));
    expect(result.current.pending).toBe(true);

    // 응답 도착 후 — pending=false
    await act(async () => {
      respond!();
      await callPromise;
    });
    expect(result.current.pending).toBe(false);
    expect(result.current.success).toBe(true);
  });
});
