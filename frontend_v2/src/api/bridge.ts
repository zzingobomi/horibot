import { decode as msgpackDecode } from "@msgpack/msgpack";
import ReconnectingWebSocket from "reconnecting-websocket";
import {
  FRAME_VERSION,
  FrameType,
  WsOp,
} from "@/types/bridge";
import type { WsOutgoing } from "@/types/bridge";
import { DEFAULT_ROBOT_ID, WS_URL } from "@/constants";
import type {
  ServiceMap,
  TopicPayloadMap,
} from "@/api/generated/contract";
// 응답 auto-cache 위해 framework store 직접 import (transport ↔ framework
// circular: top-level 사용 X, callService 런타임 안에서만 호출 → safe).
import { useFrameworkStore } from "@/framework/store";

// robot-scoped template (`srv/.../{robot_id}/...` 등) auto-expand.
function expandTopicKey(key: string, robotId: string): string {
  return key.includes("{robot_id}")
    ? key.replace(/\{robot_id\}/g, robotId)
    : key;
}

export function topicFor(template: string, robotId: string = DEFAULT_ROBOT_ID): string {
  return expandTopicKey(template, robotId);
}

type TopicCallback = (data: Record<string, unknown>) => void;
// backend_v2 wire 에선 topic payload 도 msgpack 통과. binary callback 은 그 msgpack-encoded
// raw bytes 받음 (decode 는 consumer 책임 — bytes field 추출 등).
type BinaryTopicCallback = (payload: ArrayBuffer) => void;
type ServiceResolver = (res: {
  success: boolean;
  message: string;
  data: Record<string, unknown>;
}) => void;

export type ServiceResponse<T> = {
  success: boolean;
  message: string;
  data: T;
};

function makeRequestId(): string {
  if (
    typeof crypto !== "undefined" &&
    typeof crypto.randomUUID === "function"
  ) {
    return crypto.randomUUID();
  }
  return `req-${Date.now().toString(36)}-${Math.random()
    .toString(36)
    .slice(2, 10)}`;
}

interface DecodedFrame {
  type: number;
  key: string;
  payload: ArrayBuffer;
}

export function decodeFrame(buf: ArrayBuffer): DecodedFrame | null {
  if (buf.byteLength < 4) return null;
  const view = new DataView(buf);
  if (view.getUint8(0) !== FRAME_VERSION) return null;
  const type = view.getUint8(1);
  const keyLen = view.getUint16(2, false);
  const headerLen = 4 + keyLen;
  if (buf.byteLength < headerLen) return null;
  const key = new TextDecoder().decode(buf.slice(4, headerLen));
  return { type, key, payload: buf.slice(headerLen) };
}

export function decodeMsgpackRecord(payload: ArrayBuffer): Record<string, unknown> {
  const decoded = msgpackDecode(new Uint8Array(payload));
  return (decoded ?? {}) as Record<string, unknown>;
}

export class BridgeClient {
  private ws: ReconnectingWebSocket | null = null;
  private topicListeners = new Map<string, Set<TopicCallback>>();
  private binaryTopicListeners = new Map<string, Set<BinaryTopicCallback>>();
  private pendingServices = new Map<string, ServiceResolver>();
  // ws 가 아직 OPEN 아닐 때(초기 연결/재연결 창) 낸 service RPC 프레임 — 버리지 않고
  // 버퍼했다가 open 시 flush. 재연결 창에서 로봇 명령(moveJ/gripper/task run 등)이
  // 조용히 유실되던 결함 방지. publish(jog 50Hz 등)는 latest-wins 라 버퍼 X
  // (재연결 후 stale 명령 재생은 오히려 위험).
  private unsentServices: Array<{ id: string; frame: string }> = [];
  private onStatusChange?: (connected: boolean) => void;
  private defaultRobotId: string = DEFAULT_ROBOT_ID;
  private defaultRobotListeners = new Set<(robotId: string) => void>();

  setDefaultRobotId(robotId: string): void {
    if (this.defaultRobotId === robotId) return;
    this.defaultRobotId = robotId;
    for (const l of this.defaultRobotListeners) l(robotId);
  }

  onDefaultRobotIdChange(listener: (robotId: string) => void): () => void {
    this.defaultRobotListeners.add(listener);
    return () => {
      this.defaultRobotListeners.delete(listener);
    };
  }

  /**
   * 외부 (framework hooks) 에서 직접 expand. robotId 미지정 = 현재 defaultRobotId.
   * placeholder 없으면 그대로.
   */
  expand(key: string, robotId?: string): string {
    return expandTopicKey(key, robotId ?? this.defaultRobotId);
  }

  private _expand(key: string): string {
    return expandTopicKey(key, this.defaultRobotId);
  }

  connect(onStatusChange?: (connected: boolean) => void): void {
    this.onStatusChange = onStatusChange;

    if (this.ws) {
      console.log("[Bridge] 이미 생성됨");
      return;
    }

    this.ws = new ReconnectingWebSocket(WS_URL, [], {
      maxRetries: Infinity,
    });
    this.ws.binaryType = "arraybuffer";

    this.ws.addEventListener("open", () => {
      console.log("[Bridge] 연결됨");
      this.onStatusChange?.(true);
      this._resubscribeAll();
      this._flushUnsentServices();
    });

    this.ws.addEventListener("message", (ev) => {
      // backend_v2 → browser : binary frame only. JSON text 안 옴.
      if (ev.data instanceof ArrayBuffer) {
        this._handleBinary(ev.data);
      } else {
        console.warn("[Bridge] 예상치 못한 text 메시지", ev.data);
      }
    });

    this.ws.addEventListener("close", () => {
      console.log("[Bridge] 연결 끊김");
      this.onStatusChange?.(false);
    });

    this.ws.addEventListener("error", (e) => {
      console.error("[Bridge] 오류", e);
    });
  }

  private _handleBinary(buf: ArrayBuffer): void {
    const frame = decodeFrame(buf);
    if (!frame) {
      console.warn("[Bridge] 바이너리 헤더 파싱 실패");
      return;
    }

    if (frame.type === FrameType.TopicData) {
      const rawCbs = this.binaryTopicListeners.get(frame.key);
      rawCbs?.forEach((cb) => cb(frame.payload));

      const jsonCbs = this.topicListeners.get(frame.key);
      if (jsonCbs && jsonCbs.size > 0) {
        try {
          const decoded = decodeMsgpackRecord(frame.payload);
          jsonCbs.forEach((cb) => cb(decoded));
        } catch (e) {
          console.error("[Bridge] topic msgpack decode 실패", frame.key, e);
        }
      }
      return;
    }

    if (frame.type === FrameType.ServiceResponse) {
      const resolver = this.pendingServices.get(frame.key);
      if (!resolver) return;
      this.pendingServices.delete(frame.key);
      try {
        const env = decodeMsgpackRecord(frame.payload);
        const data = (env.data ?? {}) as Record<string, unknown>;
        resolver({ success: true, message: "", data });
      } catch (e) {
        resolver({
          success: false,
          message: `service response decode 실패: ${String(e)}`,
          data: {},
        });
      }
      return;
    }

    if (frame.type === FrameType.ServiceError) {
      const resolver = this.pendingServices.get(frame.key);
      if (!resolver) return;
      this.pendingServices.delete(frame.key);
      try {
        const err = decodeMsgpackRecord(frame.payload) as {
          type?: string;
          message?: string;
        };
        const message = err.type
          ? `${err.type}: ${err.message ?? ""}`
          : err.message ?? "서비스 오류";
        resolver({ success: false, message, data: {} });
      } catch (e) {
        resolver({
          success: false,
          message: `service error decode 실패: ${String(e)}`,
          data: {},
        });
      }
      return;
    }

    console.warn("[Bridge] 알 수 없는 frame type", frame.type, frame.key);
  }

  private _resubscribeAll(): void {
    console.log("[Bridge] 모든 토픽 재구독");

    for (const topic of this.topicListeners.keys()) {
      this._send({ op: WsOp.Subscribe, topic });
    }
    for (const topic of this.binaryTopicListeners.keys()) {
      this._send({ op: WsOp.Subscribe, topic });
    }
  }

  subscribe<T extends keyof TopicPayloadMap>(
    topic: T,
    callback: (data: TopicPayloadMap[T]) => void,
  ): () => void;
  subscribe(topic: string, callback: TopicCallback): () => void;
  subscribe(topic: string, callback: TopicCallback): () => void {
    const expanded = this._expand(topic);

    if (!this.topicListeners.has(expanded)) {
      this.topicListeners.set(expanded, new Set());
    }

    this.topicListeners.get(expanded)!.add(callback);

    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this._send({ op: WsOp.Subscribe, topic: expanded });
    }

    return () => {
      const cbs = this.topicListeners.get(expanded);
      if (!cbs) return;

      cbs.delete(callback);

      if (cbs.size === 0) {
        this.topicListeners.delete(expanded);
        const hasBinary = this.binaryTopicListeners.has(expanded);
        if (!hasBinary && this.ws && this.ws.readyState === WebSocket.OPEN) {
          this._send({ op: WsOp.Unsubscribe, topic: expanded });
        }
      }
    };
  }

  subscribeBinary(topic: string, callback: BinaryTopicCallback): () => void {
    const expanded = this._expand(topic);

    if (!this.binaryTopicListeners.has(expanded)) {
      this.binaryTopicListeners.set(expanded, new Set());
    }

    this.binaryTopicListeners.get(expanded)!.add(callback);

    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this._send({ op: WsOp.Subscribe, topic: expanded });
    }

    return () => {
      const cbs = this.binaryTopicListeners.get(expanded);
      if (!cbs) return;

      cbs.delete(callback);

      if (cbs.size === 0) {
        this.binaryTopicListeners.delete(expanded);
        const hasJson = this.topicListeners.has(expanded);
        if (!hasJson && this.ws && this.ws.readyState === WebSocket.OPEN) {
          this._send({ op: WsOp.Unsubscribe, topic: expanded });
        }
      }
    };
  }

  publish<T extends keyof TopicPayloadMap>(
    topic: T,
    data: TopicPayloadMap[T],
    robotId?: string,
  ): void;
  publish(topic: string, data: Record<string, unknown>, robotId?: string): void;
  publish(topic: string, data: unknown, robotId?: string): void {
    this._send({
      op: WsOp.Publish,
      topic: this.expand(topic, robotId),
      data: data as Record<string, unknown>,
    });
  }

  callService<K extends keyof ServiceMap>(
    key: K,
    data: ServiceMap[K]["req"],
    options?: { timeoutMs?: number; robotId?: string },
  ): Promise<ServiceResponse<ServiceMap[K]["res"]>>;
  callService(
    key: string,
    data: Record<string, unknown>,
    options?: { timeoutMs?: number; robotId?: string },
  ): Promise<ServiceResponse<Record<string, unknown>>>;
  callService(
    key: string,
    data: unknown,
    options?: { timeoutMs?: number; robotId?: string },
  ): Promise<ServiceResponse<unknown>> {
    const timeoutMs = options?.timeoutMs ?? 5000;
    const expanded = this.expand(key, options?.robotId);
    const prev = useFrameworkStore.getState().serviceData[expanded];
    useFrameworkStore.getState().setServiceData(expanded, {
      success: prev?.success ?? false,
      message: prev?.message ?? "",
      data: prev?.data ?? null,
      timestamp: prev?.timestamp ?? 0,
      pending: true,
    });
    return new Promise((resolve) => {
      const request_id = makeRequestId();
      const cacheAndResolve: ServiceResolver = (res) => {
        useFrameworkStore.getState().setServiceData(expanded, {
          success: res.success,
          message: res.message,
          data: res.data,
          timestamp: Date.now(),
          pending: false,
        });
        resolve(res);
      };
      this.pendingServices.set(request_id, cacheAndResolve);
      // Bridge = 순수 transport — 키 확장(라우팅)만. robot-agnostic 서비스의
      // robot_id 는 req 필드 (호출자가 data 에 넣음, 타입이 강제) — 여기서 주입 X.
      // timeout 은 wire 로 전파 — bridge 의 zenoh call 이 같은 상한을 쓰게
      // (장시간 서비스가 bridge 기본 5s 에 잘리던 회귀 방지).
      const frame: WsOutgoing = {
        op: WsOp.Service,
        key: expanded,
        request_id,
        data: data as Record<string, unknown>,
        timeout_s: timeoutMs / 1000,
      };
      // ws 가 CONNECTING(초기/재연결) 이면 _send 가 drop → RPC 프레임을 버퍼했다가
      // open 시 flush. (frame 을 직렬화해 stash — open 시점에 재직렬화 불필요)
      if (!this._send(frame)) {
        this.unsentServices.push({ id: request_id, frame: JSON.stringify(frame) });
      }

      // backend 도 5s default timeout — frontend 가 safety net
      setTimeout(() => {
        if (this.pendingServices.has(request_id)) {
          this.pendingServices.delete(request_id);
          cacheAndResolve({
            success: false,
            message: "서비스 응답 타임아웃",
            data: {} as unknown as Record<string, unknown>,
          });
        }
      }, timeoutMs);
    });
  }

  /** ws 가 OPEN 이면 전송하고 true, 아니면(연결 전/재연결 중) 전송 안 하고 false. */
  private _send(msg: WsOutgoing): boolean {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(msg));
      return true;
    }
    return false;
  }

  /** open 시 호출 — 연결 창에 쌓인 service RPC 프레임 flush. 이미 timeout/resolve 된
   *  요청(pendingServices 에 없음)은 재전송 X — 중복 side-effect 방지. */
  private _flushUnsentServices(): void {
    if (this.unsentServices.length === 0) return;
    const items = this.unsentServices;
    this.unsentServices = [];
    for (const it of items) {
      if (!this.pendingServices.has(it.id)) continue;
      if (this.ws && this.ws.readyState === WebSocket.OPEN) {
        this.ws.send(it.frame);
      }
    }
  }

  disconnect(): void {
    if (!this.ws) return;

    this.ws.close(1000, "client disconnect");
    this.ws = null;
  }
}

export const bridge = new BridgeClient();
