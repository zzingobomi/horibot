// backend_v2 WS wire (frontend_v2.md §3 + backend_v2_modules.md §8.6 의 relay).
// browser → bridge : JSON text {op, ...}
// bridge → browser : binary frame [u8 ver=1][u8 type][u16 BE key_len][key utf8][payload]
//   type=1 topic_data       : key=topic,      payload=msgpack
//   type=2 service_response : key=request_id, payload=msgpack {timestamp, data}
//   type=3 service_error    : key=request_id, payload=msgpack {type, message}

export const WsOp = {
  Subscribe: "subscribe",
  Unsubscribe: "unsubscribe",
  Publish: "publish",
  Service: "service",
} as const;

export type WsOp = (typeof WsOp)[keyof typeof WsOp];

export type WsOutgoing =
  | { op: typeof WsOp.Subscribe; topic: string }
  | { op: typeof WsOp.Unsubscribe; topic: string }
  | { op: typeof WsOp.Publish; topic: string; data: Record<string, unknown> }
  | {
      op: typeof WsOp.Service;
      key: string;
      request_id: string;
      data: Record<string, unknown>;
      robot_id?: string;
    };

export const FrameType = {
  TopicData: 1,
  ServiceResponse: 2,
  ServiceError: 3,
} as const;

export type FrameType = (typeof FrameType)[keyof typeof FrameType];

export const FRAME_VERSION = 1;
