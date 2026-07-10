// frontend_v2.md §12.2 useCapability — 2 invariant.

import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { renderHook, waitFor } from "@testing-library/react";
import { encode as msgpackEncode } from "@msgpack/msgpack";
import { Server } from "mock-socket";
import { WS_URL } from "@/constants";
import { FRAME_VERSION, FrameType, WsOp } from "@/types/bridge";
import { bridge } from "@/api/bridge";
import { useFrameworkStore } from "./store";
import { _resetCapabilityCache, useCapability } from "./capability";

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
  _resetCapabilityCache();
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
      useFrameworkStore.getState().setBridgeConnected(connected);
      if (connected) resolve();
    });
  });
}

describe("useCapability", () => {
  // spec frontend_v2.md §12.2 — invariant: mount → 1회 service call
  it("boot 1회 fetch — mount 시 service call 1회만", async () => {
    let callCount = 0;
    server.on("connection", (socket) => {
      socket.on("message", (raw) => {
        const msg = JSON.parse(raw as string);
        if (msg.op === WsOp.Service) {
          callCount++;
          const env = msgpackEncode({
            timestamp: 0,
            data: { flags: ["torque_toggle", "reboot"] },
          });
          socket.send(encodeFrame(FrameType.ServiceResponse, msg.request_id, env));
        }
      });
    });
    await waitOpen();

    const { result } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useCapability("srv/motor/{robot_id}/capabilities" as any, { robotId: "so101_6dof_0" }),
    );

    await waitFor(() => expect(result.current.value).not.toBeNull());
    expect(callCount).toBe(1);
    expect(result.current.value).toEqual({ flags: ["torque_toggle", "reboot"] });
  });

  // spec frontend_v2.md §12.2 — invariant: re-mount 시 fetch 안 함 (cache hit)
  it("module-cache — 두 hook 박혀도 fetch 1회만 호출 (re-mount cache hit)", async () => {
    let callCount = 0;
    server.on("connection", (socket) => {
      socket.on("message", (raw) => {
        const msg = JSON.parse(raw as string);
        if (msg.op === WsOp.Service) {
          callCount++;
          const env = msgpackEncode({ timestamp: 0, data: { flags: ["depth"] } });
          socket.send(encodeFrame(FrameType.ServiceResponse, msg.request_id, env));
        }
      });
    });
    await waitOpen();

    const { result: a } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useCapability("srv/camera/{robot_id}/capabilities" as any, { robotId: "so101_6dof_0" }),
    );
    await waitFor(() => expect(a.current.value).not.toBeNull());

    const { result: b } = renderHook(() =>
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      useCapability("srv/camera/{robot_id}/capabilities" as any, { robotId: "so101_6dof_0" }),
    );

    // b 자리 mount 즉시 cache hit — value 박혀있음
    expect(b.current.value).toEqual({ flags: ["depth"] });
    expect(callCount).toBe(1);
  });
});
