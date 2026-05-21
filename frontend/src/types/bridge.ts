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
export interface WsSubscribe {
  type: typeof WsMsgType.Subscribe;
  topic: string;
}

export interface WsUnsubscribe {
  type: typeof WsMsgType.Unsubscribe;
  topic: string;
}

export interface WsPublish {
  type: typeof WsMsgType.Publish;
  topic: string;
  data: Record<string, unknown>;
}

export interface WsService {
  type: typeof WsMsgType.Service;
  key: string;
  request_id: string;
  data: Record<string, unknown>;
  timeout?: number; // 초 단위. 미지정 시 백엔드 기본값(5초)
}

export type WsOutgoing = WsSubscribe | WsUnsubscribe | WsPublish | WsService;

// ─── Bridge → Frontend ────────────────────────────────────────
export interface WsTopicData {
  type: typeof WsMsgType.TopicData;
  topic: string;
  data: Record<string, unknown>;
}

export interface WsServiceResponse {
  type: typeof WsMsgType.ServiceResponse;
  request_id: string;
  success: boolean;
  message: string;
  data: Record<string, unknown>;
}

export interface WsError {
  type: typeof WsMsgType.Error;
  message: string;
}

export type WsIncoming = WsTopicData | WsServiceResponse | WsError;
