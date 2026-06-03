import ReconnectingWebSocket from "reconnecting-websocket";
import { WsMsgType } from "@/types/bridge";
import type { WsIncoming, WsOutgoing } from "@/types/bridge";
import { WS_URL } from "@/constants";
import type {
  ServiceMap,
  TopicPayloadMap,
} from "@/api/generated/contract";

type TopicCallback = (data: Record<string, unknown>) => void;
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

// 바이너리 프레임: [u8 v=1][u8 type=1][u16 BE topic_len][topic UTF-8][payload]
const BIN_VERSION = 1;
const BIN_TYPE_TOPIC_DATA = 1;

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

function decodeBinaryTopic(
  buf: ArrayBuffer
): { topic: string; payload: ArrayBuffer } | null {
  if (buf.byteLength < 4) return null;
  const view = new DataView(buf);
  if (view.getUint8(0) !== BIN_VERSION) return null;
  if (view.getUint8(1) !== BIN_TYPE_TOPIC_DATA) return null;
  const topicLen = view.getUint16(2, false);
  const headerLen = 4 + topicLen;
  if (buf.byteLength < headerLen) return null;
  const topic = new TextDecoder().decode(buf.slice(4, headerLen));
  return { topic, payload: buf.slice(headerLen) };
}

class BridgeClient {
  private ws: ReconnectingWebSocket | null = null;
  private topicListeners = new Map<string, Set<TopicCallback>>();
  private binaryTopicListeners = new Map<string, Set<BinaryTopicCallback>>();
  private pendingServices = new Map<string, ServiceResolver>();
  private onStatusChange?: (connected: boolean) => void;

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
    });

    this.ws.addEventListener("message", (ev) => {
      if (typeof ev.data === "string") {
        try {
          const msg = JSON.parse(ev.data) as WsIncoming;
          this._handleIncoming(msg);
        } catch (e) {
          console.error("[Bridge] 메시지 파싱 오류", e);
        }
      } else if (ev.data instanceof ArrayBuffer) {
        this._handleBinary(ev.data);
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

  private _handleIncoming(msg: WsIncoming): void {
    if (msg.type === WsMsgType.TopicData) {
      const cbs = this.topicListeners.get(msg.topic);
      cbs?.forEach((cb) => cb(msg.data));
    } else if (msg.type === WsMsgType.ServiceResponse) {
      const resolve = this.pendingServices.get(msg.request_id);
      if (resolve) {
        resolve({ success: msg.success, message: msg.message, data: msg.data });
        this.pendingServices.delete(msg.request_id);
      }
    } else if (msg.type === WsMsgType.Error) {
      console.error("[Bridge] 서버 오류:", msg.message);
    }
  }

  private _handleBinary(buf: ArrayBuffer): void {
    const decoded = decodeBinaryTopic(buf);
    if (!decoded) {
      console.warn("[Bridge] 바이너리 헤더 파싱 실패");
      return;
    }
    const cbs = this.binaryTopicListeners.get(decoded.topic);
    cbs?.forEach((cb) => cb(decoded.payload));
  }

  private _resubscribeAll(): void {
    console.log("[Bridge] 모든 토픽 재구독");

    for (const topic of this.topicListeners.keys()) {
      this._send({ type: WsMsgType.Subscribe, topic });
    }
    for (const topic of this.binaryTopicListeners.keys()) {
      this._send({ type: WsMsgType.Subscribe, topic });
    }
  }

  subscribe<T extends keyof TopicPayloadMap>(
    topic: T,
    callback: (data: TopicPayloadMap[T]) => void,
  ): () => void;
  subscribe(topic: string, callback: TopicCallback): () => void;
  subscribe(topic: string, callback: TopicCallback): () => void {
    console.log(`[Bridge] 구독 요청: ${topic}`);

    if (!this.topicListeners.has(topic)) {
      this.topicListeners.set(topic, new Set());
    }

    this.topicListeners.get(topic)!.add(callback);

    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this._send({
        type: WsMsgType.Subscribe,
        topic,
      });
    }

    return () => {
      const cbs = this.topicListeners.get(topic);
      if (!cbs) return;

      cbs.delete(callback);

      if (cbs.size === 0) {
        this.topicListeners.delete(topic);

        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
          this._send({
            type: WsMsgType.Unsubscribe,
            topic,
          });
        }
      }
    };
  }

  subscribeBinary(topic: string, callback: BinaryTopicCallback): () => void {
    console.log(`[Bridge] 바이너리 구독 요청: ${topic}`);

    if (!this.binaryTopicListeners.has(topic)) {
      this.binaryTopicListeners.set(topic, new Set());
    }

    this.binaryTopicListeners.get(topic)!.add(callback);

    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this._send({ type: WsMsgType.Subscribe, topic });
    }

    return () => {
      const cbs = this.binaryTopicListeners.get(topic);
      if (!cbs) return;

      cbs.delete(callback);

      if (cbs.size === 0) {
        this.binaryTopicListeners.delete(topic);

        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
          this._send({ type: WsMsgType.Unsubscribe, topic });
        }
      }
    };
  }

  publish<T extends keyof TopicPayloadMap>(
    topic: T,
    data: TopicPayloadMap[T],
  ): void;
  publish(topic: string, data: Record<string, unknown>): void;
  publish(topic: string, data: unknown): void {
    this._send({
      type: WsMsgType.Publish,
      topic,
      data: data as Record<string, unknown>,
    });
  }

  callService<K extends keyof ServiceMap>(
    key: K,
    data: ServiceMap[K]["req"],
    options?: { timeoutMs?: number },
  ): Promise<ServiceResponse<ServiceMap[K]["res"]>>;
  callService(
    key: string,
    data: Record<string, unknown>,
    options?: { timeoutMs?: number },
  ): Promise<ServiceResponse<Record<string, unknown>>>;
  callService(
    key: string,
    data: unknown,
    options?: { timeoutMs?: number },
  ): Promise<ServiceResponse<unknown>> {
    const timeoutMs = options?.timeoutMs ?? 5000;
    return new Promise((resolve) => {
      const request_id = makeRequestId();
      this.pendingServices.set(
        request_id,
        resolve as ServiceResolver,
      );
      this._send({
        type: WsMsgType.Service,
        key,
        request_id,
        data: data as Record<string, unknown>,
        timeout: timeoutMs / 1000,
      });

      setTimeout(() => {
        if (this.pendingServices.has(request_id)) {
          this.pendingServices.delete(request_id);
          resolve({
            success: false,
            message: "서비스 응답 타임아웃",
            data: {} as unknown,
          });
        }
      }, timeoutMs);
    });
  }

  private _send(msg: WsOutgoing): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(msg));
    }
  }

  disconnect(): void {
    if (!this.ws) return;

    this.ws.close(1000, "client disconnect");
    this.ws = null;
  }
}

export const bridge = new BridgeClient();
