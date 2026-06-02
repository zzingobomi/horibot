import logging
import threading
import time
import json
from typing import Callable, TypeVar, overload

import zenoh
from pydantic import BaseModel

from core.zenoh_session import ZenohSession
from core.topic_map import Topic
from core.messages.base import ServiceRequest, ServiceResponse
from core.messages.system import Heartbeat, LogMessage

logger = logging.getLogger(__name__)


# Typed callback / service handler 용 TypeVar.
M = TypeVar("M", bound=BaseModel)
ReqT = TypeVar("ReqT", bound=BaseModel)
ResT = TypeVar("ResT", bound=BaseModel)


class BaseNode:
    def __init__(self, node_name: str):
        self.node_name = node_name
        self.session: zenoh.Session = ZenohSession.get()
        self._subscribers: list[zenoh.Subscriber] = []
        self._queryables: list[zenoh.Queryable] = []
        self._running = False
        self._heartbeat_thread: threading.Thread | None = None

    # ─── Subscriber ──────────────────────────────────────────

    @overload
    def create_subscriber(
        self, topic: str, callback: Callable[[dict], None]
    ) -> None: ...

    @overload
    def create_subscriber(
        self, topic: str, model_cls: type[M], callback: Callable[[M], None]
    ) -> None: ...

    def create_subscriber(self, topic, arg2, callback=None):  # type: ignore[no-untyped-def]
        """Subscribe — legacy(dict) 또는 typed(model_cls) 두 형태 지원.

        legacy: `create_subscriber(topic, callback)` — callback(dict).
        typed:  `create_subscriber(topic, ModelCls, callback)` — callback(ModelCls).

        typed 형태에서 JSON 파싱 + pydantic validation 실패 시 ValidationError 가
        로그에만 남고 콜백 호출 X — drift / version mismatch 즉시 발견 가능.
        """
        if callback is None:
            # legacy: arg2 가 dict 콜백
            legacy_cb: Callable[[dict], None] = arg2

            def _handler(sample: zenoh.Sample) -> None:
                try:
                    data = json.loads(sample.payload.to_bytes())
                    legacy_cb(data)
                except Exception as e:
                    logger.error(
                        f"[{self.node_name}] subscriber 처리 오류 ({topic}): {e}"
                    )
        else:
            # typed: arg2 가 model class
            model_cls: type[BaseModel] = arg2
            typed_cb: Callable[[BaseModel], None] = callback

            def _handler(sample: zenoh.Sample) -> None:
                try:
                    obj = model_cls.model_validate_json(sample.payload.to_bytes())
                    typed_cb(obj)
                except Exception as e:
                    logger.error(
                        f"[{self.node_name}] typed subscriber 검증 실패 ({topic}): {e}"
                    )

        sub = self.session.declare_subscriber(topic, _handler)
        self._subscribers.append(sub)
        logger.debug(f"[{self.node_name}] subscriber 등록: {topic}")

    def create_raw_subscriber(
        self, topic: str, callback: Callable[[bytes], None]
    ) -> None:
        def _handler(sample: zenoh.Sample) -> None:
            try:
                callback(sample.payload.to_bytes())
            except Exception as e:
                logger.error(f"[{self.node_name}] raw subscriber 오류 ({topic}): {e}")

        sub = self.session.declare_subscriber(topic, _handler)
        self._subscribers.append(sub)
        logger.debug(f"[{self.node_name}] raw subscriber 등록: {topic}")

    # ─── Service (Queryable / 서버) ──────────────────────────

    @overload
    def create_service(
        self, key: str, handler: Callable[[dict], dict]
    ) -> None: ...

    @overload
    def create_service(
        self,
        key: str,
        req_cls: type[ReqT],
        res_cls: type[ResT],
        handler: Callable[[ServiceRequest[ReqT]], ServiceResponse[ResT]],
    ) -> None: ...

    def create_service(self, key, arg2, arg3=None, arg4=None):  # type: ignore[no-untyped-def]
        """Service 등록 — legacy(dict) 또는 typed(ReqCls/ResCls) 두 형태 지원.

        legacy: `create_service(key, handler)` — handler(dict) -> dict.
        typed:  `create_service(key, ReqCls, ResCls, handler)` —
                handler(ServiceRequest[ReqCls]) -> ServiceResponse[ResCls].

        typed handler 에서 req 파싱 실패 / handler 자체 에러 시 success=False 응답
        반환. wire 형태는 legacy 와 동일 (`{success, message, data}`) — 양쪽
        호환 유지.
        """
        if arg3 is None and arg4 is None:
            # legacy
            legacy_handler: Callable[[dict], dict] = arg2

            def _handler(query: zenoh.Query) -> None:
                try:
                    payload = query.payload
                    req = json.loads(payload.to_bytes()) if payload else {}
                    res = legacy_handler(req)
                    query.reply(key, json.dumps(res).encode())
                except Exception as e:
                    logger.error(
                        f"[{self.node_name}] service 처리 오류 ({key}): {e}"
                    )
                    err = {"success": False, "message": str(e), "data": {}}
                    query.reply(key, json.dumps(err).encode())
        else:
            # typed: arg2=ReqCls, arg3=ResCls, arg4=handler. arg4 가 None 이면
            # 호출 형태가 잘못된 것 — overload 가 막아주지만 런타임 가드.
            if arg4 is None:
                raise TypeError("typed create_service: handler 인자 필요")
            req_cls: type[BaseModel] = arg2
            typed_handler: Callable[[ServiceRequest], ServiceResponse] = arg4
            req_envelope_cls = ServiceRequest[req_cls]  # type: ignore[valid-type]

            def _handler(query: zenoh.Query) -> None:
                try:
                    payload = query.payload
                    if payload is None:
                        # 빈 페이로드 — timestamp/data 누락이라 typed 경로에선 에러.
                        raise ValueError("typed service 호출에 페이로드 없음")
                    req = req_envelope_cls.model_validate_json(payload.to_bytes())
                    res = typed_handler(req)
                    query.reply(key, res.model_dump_json().encode())
                except Exception as e:
                    logger.error(
                        f"[{self.node_name}] typed service 처리 오류 ({key}): {e}"
                    )
                    err = ServiceResponse(success=False, message=str(e), data=None)
                    query.reply(key, err.model_dump_json().encode())

        queryable = self.session.declare_queryable(key, _handler)
        self._queryables.append(queryable)
        logger.debug(f"[{self.node_name}] service 등록: {key}")

    # ─── Service Client (Get / 클라이언트) ───────────────────

    @overload
    def call_service(
        self, key: str, data: dict, timeout: float = 5.0
    ) -> dict: ...

    @overload
    def call_service(
        self,
        key: str,
        data: BaseModel,
        res_cls: type[ResT],
        timeout: float = 5.0,
    ) -> "ServiceResponse[ResT]": ...

    def call_service(self, key, data, *args, **kwargs):  # type: ignore[no-untyped-def]
        """Zenoh Get 으로 서비스 호출.

        legacy: `call_service(key, dict_data, timeout=5.0)` -> dict.
        typed:  `call_service(key, ReqModel(...), ResCls, timeout=5.0)`
                -> ServiceResponse[ResCls].

        타임아웃 / 응답 없음 / 에러 시 success=False (legacy) 또는
        ServiceResponse(success=False, data=None) (typed) 반환.
        """
        timeout: float = kwargs.pop("timeout", 5.0)

        if isinstance(data, BaseModel):
            # typed path: 3번째 positional 또는 'res_cls' kwarg 가 res_cls
            res_cls: type[BaseModel] | None = kwargs.pop("res_cls", None)
            if res_cls is None and args:
                res_cls = args[0]
                args = args[1:]
            if res_cls is None:
                raise TypeError(
                    "typed call_service 는 res_cls 인자 필요 "
                    "(call_service(key, ReqModel(...), ResCls, ...))"
                )
            if args:
                # 마지막 positional 이 timeout 일 수도
                timeout = args[0]
            return self._call_service_typed(key, data, res_cls, timeout)

        # legacy dict path
        if args:
            timeout = args[0]
        return self._call_service_dict(key, data, timeout)

    def _call_service_dict(
        self, key: str, data: dict, timeout: float
    ) -> dict:
        payload = json.dumps(
            {"timestamp": time.time(), "data": data}
        ).encode()

        try:
            replies = self.session.get(key, payload=payload, timeout=timeout)
            for reply in replies:
                if reply.ok is not None:
                    return json.loads(reply.ok.payload.to_bytes())
                err = reply.err
                msg = (
                    err.payload.to_string()
                    if err is not None and err.payload is not None
                    else "서비스 err reply"
                )
                logger.warning(
                    f"[{self.node_name}] service err reply: {key} — {msg}"
                )
                return {"success": False, "message": msg, "data": {}}
            logger.warning(f"[{self.node_name}] service 응답 없음: {key}")
            return {"success": False, "message": "응답 없음", "data": {}}
        except Exception as e:
            logger.error(f"[{self.node_name}] call_service 오류 ({key}): {e}")
            return {"success": False, "message": str(e), "data": {}}

    def _call_service_typed(
        self,
        key: str,
        data: BaseModel,
        res_cls: type[BaseModel],
        timeout: float,
    ) -> ServiceResponse:
        # ServiceRequest envelope 로 감싸 발신.
        req_obj = ServiceRequest(timestamp=time.time(), data=data)
        payload = req_obj.model_dump_json().encode()
        res_envelope_cls = ServiceResponse[res_cls]  # type: ignore[valid-type]

        try:
            replies = self.session.get(key, payload=payload, timeout=timeout)
            for reply in replies:
                if reply.ok is not None:
                    return res_envelope_cls.model_validate_json(
                        reply.ok.payload.to_bytes()
                    )
                err = reply.err
                msg = (
                    err.payload.to_string()
                    if err is not None and err.payload is not None
                    else "서비스 err reply"
                )
                logger.warning(
                    f"[{self.node_name}] service err reply: {key} — {msg}"
                )
                return ServiceResponse(success=False, message=msg, data=None)
            logger.warning(f"[{self.node_name}] service 응답 없음: {key}")
            return ServiceResponse(success=False, message="응답 없음", data=None)
        except Exception as e:
            logger.error(f"[{self.node_name}] call_service 오류 ({key}): {e}")
            return ServiceResponse(success=False, message=str(e), data=None)

    # ─── Lifecycle ───────────────────────────────────────────

    def start(self) -> None:
        self._running = True
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name=f"{self.node_name}-heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()
        logger.info(f"[{self.node_name}] 시작됨")

    def stop(self) -> None:
        self._running = False
        for sub in self._subscribers:
            sub.undeclare()
        for q in self._queryables:
            q.undeclare()
        self._subscribers.clear()
        self._queryables.clear()
        logger.info(f"[{self.node_name}] 종료됨")

    def spin(self) -> None:
        """노드를 블로킹으로 실행. 스레드에서 호출할 것."""
        self.start()
        try:
            while self._running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    # ─── Publisher ───────────────────────────────────────────

    def publish(self, topic: str, data: dict | BaseModel) -> None:
        """JSON 직렬화 후 토픽 발행. BaseModel 이면 model_dump_json, 아니면 dict."""
        if isinstance(data, BaseModel):
            payload = data.model_dump_json().encode()
        else:
            payload = json.dumps(data).encode()
        self.session.put(topic, payload)

    # ─── Heartbeat ───────────────────────────────────────────

    def _heartbeat_loop(self) -> None:
        while self._running:
            self.publish(
                Topic.SYSTEM_HEARTBEAT,
                Heartbeat(
                    node=self.node_name,
                    timestamp=time.time(),
                    status="ok",
                ),
            )
            time.sleep(1.0)

    # ─── Log ─────────────────────────────────────────────────

    def log(self, level: str, msg: str) -> None:
        # SYSTEM_LOG 페이로드. level 은 typed Literal — 'debug'/'info'/'warning'/'error'
        # 외 값이 들어오면 pydantic ValidationError 던짐. drift 방지.
        self.publish(
            Topic.SYSTEM_LOG,
            LogMessage(
                node=self.node_name,
                timestamp=time.time(),
                level=level,  # type: ignore[arg-type]
                message=msg,
            ),
        )
        getattr(logger, level, logger.info)(f"[{self.node_name}] {msg}")
