// frontend_v2.md §12.2 — bridge.ts 의 6 invariant 검증.
// 단순 PASS 박는 test 박지 X — 각 it 의 docstring 에 spec ref + invariant 명시.

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { encode as msgpackEncode } from "@msgpack/msgpack";
import { Server } from "mock-socket";
import { WS_URL } from "@/constants";
import { FRAME_VERSION, FrameType, WsOp } from "@/types/bridge";
import { BridgeClient, decodeFrame, decodeMsgpackRecord } from "./bridge";
import { useFrameworkStore } from "@/framework/store";

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

describe("decodeFrame — binary frame parse", () => {
  // spec frontend_v2.md §12.2 — invariant: [u8 ver][u8 type][u16 BE key_len][key utf8][payload]
  it("valid frame type=1 → {type, key, payload} 정확히 추출", () => {
    const payload = new Uint8Array([0x82, 0xa1, 0x61, 0x01]); // msgpack {a:1}
    const buf = encodeFrame(FrameType.TopicData, "horibot/test", payload);
    const decoded = decodeFrame(buf);
    expect(decoded).not.toBeNull();
    expect(decoded!.type).toBe(FrameType.TopicData);
    expect(decoded!.key).toBe("horibot/test");
    expect(new Uint8Array(decoded!.payload)).toEqual(payload);
  });

  it("version mismatch (v=2) → null (graceful, throw 박지 X)", () => {
    const buf = new ArrayBuffer(8);
    const view = new DataView(buf);
    view.setUint8(0, 2); // 잘못된 version
    view.setUint8(1, 1);
    view.setUint16(2, 0, false);
    expect(decodeFrame(buf)).toBeNull();
  });

  it("header < 4 bytes → null (graceful)", () => {
    expect(decodeFrame(new ArrayBuffer(2))).toBeNull();
  });

  it("key_len mismatch (header 박았는데 payload 더 짧음) → null", () => {
    const buf = new ArrayBuffer(5); // header 4 + key_len=10 박는데 실 1 byte
    const view = new DataView(buf);
    view.setUint8(0, FRAME_VERSION);
    view.setUint8(1, 1);
    view.setUint16(2, 10, false);
    expect(decodeFrame(buf)).toBeNull();
  });
});

describe("decodeMsgpackRecord — msgpack round-trip", () => {
  // spec frontend_v2.md §12.2 — invariant: msgpack encode → decode 후 동일 object
  it("encode → decode round-trip — robot_id / seq / timestamp_unix / nested object 보존", () => {
    const original = {
      robot_id: "so101_6dof_0",
      seq: 42,
      timestamp_unix: 1719750000.123,
      joints: [0.1, -0.2, 0.3],
      nested: { foo: "bar", n: 7 },
    };
    const encoded = msgpackEncode(original);
    const decoded = decodeMsgpackRecord(
      encoded.buffer.slice(encoded.byteOffset, encoded.byteOffset + encoded.byteLength),
    );
    expect(decoded).toEqual(original);
  });
});

describe("BridgeClient — service shim + timeout + reconnect", () => {
  let server: Server;
  let client: BridgeClient;

  beforeEach(() => {
    // store isolation — pending / cache 모두 fresh
    useFrameworkStore.setState({
      topicData: {},
      serviceData: {},
      bridgeConnected: false,
    });
    server = new Server(WS_URL);
    client = new BridgeClient();
  });

  afterEach(() => {
    server.stop();
    vi.useRealTimers();
  });

  function waitOpen(): Promise<void> {
    return new Promise((resolve) => {
      client.connect((connected) => {
        if (connected) resolve();
      });
    });
  }

  it("service type=2 → success:true + data 추출 (backend envelope {timestamp, data})", async () => {
    // spec frontend_v2.md §12.2 — invariant: type=2 → {success:true, data:env.data}
    server.on("connection", (socket) => {
      socket.on("message", (raw) => {
        const msg = JSON.parse(raw as string);
        if (msg.op === WsOp.Service) {
          const env = msgpackEncode({
            timestamp: 1.0,
            data: { ok: true, value: 42 },
          });
          const frame = encodeFrame(FrameType.ServiceResponse, msg.request_id, env);
          socket.send(frame);
        }
      });
    });
    await waitOpen();
    const res = await client.callService("srv/test/echo", { x: 1 });
    expect(res.success).toBe(true);
    expect(res.message).toBe("");
    expect(res.data).toEqual({ ok: true, value: 42 });
  });

  it("service type=3 → success:false + message 'type: msg' (backend exception)", async () => {
    // spec frontend_v2.md §12.2 — invariant: type=3 → {success:false, message:`${type}: ${msg}`}
    server.on("connection", (socket) => {
      socket.on("message", (raw) => {
        const msg = JSON.parse(raw as string);
        if (msg.op === WsOp.Service) {
          const errPayload = msgpackEncode({
            type: "NotFound",
            message: "result 10 missing",
          });
          const frame = encodeFrame(FrameType.ServiceError, msg.request_id, errPayload);
          socket.send(frame);
        }
      });
    });
    await waitOpen();
    const res = await client.callService("srv/test/fail", {});
    expect(res.success).toBe(false);
    expect(res.message).toBe("NotFound: result 10 missing");
    expect(res.data).toEqual({});
  });

  it("ws CONNECTING 중 낸 service 호출 → drop 아니라 버퍼 → open 시 flush (재연결 창 명령 유실 방지)", async () => {
    // spec frontend_v2.md §12.2 — invariant: OPEN 전 낸 RPC 프레임은 버려지지 않고
    // open 시 flush 되어 정상 resolve. (2026-07-07 tasks e2e 회귀: 파싱 클릭이
    // ws CONNECTING 창에 걸려 프레임 silent drop → 5s timeout. 근본 = 이 버퍼 부재.)
    server.on("connection", (socket) => {
      socket.on("message", (raw) => {
        const msg = JSON.parse(raw as string);
        if (msg.op === WsOp.Service) {
          const env = msgpackEncode({ timestamp: 1.0, data: { ok: true, v: 7 } });
          socket.send(encodeFrame(FrameType.ServiceResponse, msg.request_id, env));
        }
      });
    });
    // open 을 await 하지 않고(=CONNECTING 상태) 즉시 service 호출 — drop 이면 timeout.
    client.connect(() => {});
    const res = await client.callService(
      "srv/test/buffered",
      { x: 1 },
      { timeoutMs: 2000 },
    );
    expect(res.success).toBe(true);
    expect(res.data).toEqual({ ok: true, v: 7 });
  });

  it("timeout safety net — backend response 안 옴 → success:false 'timeout'", async () => {
    // spec frontend_v2.md §12.2 — invariant: setTimeout 가 backend silent 시 fail-resolve
    server.on("connection", (socket) => {
      // backend silent — message 받아도 응답 박지 X
      socket.on("message", () => {});
    });
    await waitOpen();
    const promise = client.callService("srv/test/silent", {}, { timeoutMs: 50 });
    const res = await promise;
    expect(res.success).toBe(false);
    expect(res.message).toMatch(/타임아웃/);
  });

  // reconnect _resubscribeAll invariant — ReconnectingWebSocket 의 timing dep
  // (reconnectionDelayGrowFactor) 때문에 L2 unit 에서 flaky. Step F5 L4 Playwright
  // 가 mock backend kill + restart 로 실 reconnect cycle 검증.
  // spec frontend_v2.md §12.2 — L4 자리.
});
