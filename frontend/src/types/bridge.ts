export const WsMsgType = {
  // Frontend → Bridge
  Subscribe: "subscribe",
  Unsubscribe: "unsubscribe",
  Publish: "publish",
  Service: "service",
  // Bridge → Frontend
  TopicData: "topic_data",
  ServiceResponse: "service_response",
  Error: "error",
} as const;

export type WsMsgType = (typeof WsMsgType)[keyof typeof WsMsgType];

// ─── Frontend → Bridge ────────────────────────────────────────
export type WsOutgoing =
  | { type: typeof WsMsgType.Subscribe; topic: string }
  | { type: typeof WsMsgType.Unsubscribe; topic: string }
  | { type: typeof WsMsgType.Publish; topic: string; data: Record<string, unknown> }
  | {
      type: typeof WsMsgType.Service;
      key: string;
      request_id: string;
      data: Record<string, unknown>;
      timeout?: number; // 초 단위. 미지정 시 백엔드 기본값(5초)
    };

// ─── Bridge → Frontend ────────────────────────────────────────
export type WsIncoming =
  | { type: typeof WsMsgType.TopicData; topic: string; data: Record<string, unknown> }
  | {
      type: typeof WsMsgType.ServiceResponse;
      request_id: string;
      success: boolean;
      message: string;
      data: Record<string, unknown>;
    }
  | { type: typeof WsMsgType.Error; message: string };
