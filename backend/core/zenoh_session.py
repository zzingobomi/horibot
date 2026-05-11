import json
import logging
from typing import Any

import zenoh

logger = logging.getLogger(__name__)


class ZenohSession:
    _session: zenoh.Session | None = None

    @classmethod
    def get(cls) -> zenoh.Session:
        if cls._session is None:
            raise RuntimeError(
                "ZenohSession이 초기화되지 않았습니다. "
                "먼저 ZenohSession.init()을 호출하세요."
            )
        return cls._session

    @classmethod
    def init(cls, cfg_dict: dict[str, Any] | None = None) -> zenoh.Session:
        if cls._session is not None:
            logger.warning("ZenohSession이 이미 초기화되어 있습니다.")
            return cls._session

        cls._session = zenoh.open(cls._build_config(cfg_dict))
        logger.info("Zenoh 세션 시작됨 (cfg=%s)", cfg_dict or "default")
        return cls._session

    @classmethod
    def close(cls) -> None:
        if cls._session is not None:
            cls._session.close()
            cls._session = None
            logger.info("Zenoh 세션 종료됨")

    @staticmethod
    def _build_config(cfg_dict: dict[str, Any] | None) -> zenoh.Config:
        z_cfg = zenoh.Config()
        if not cfg_dict:
            return z_cfg

        mode = cfg_dict.get("mode")
        if mode:
            z_cfg.insert_json5("mode", json.dumps(mode))

        connect = cfg_dict.get("connect") or []
        if connect:
            z_cfg.insert_json5("connect/endpoints", json.dumps(list(connect)))

        listen = cfg_dict.get("listen") or []
        if listen:
            z_cfg.insert_json5("listen/endpoints", json.dumps(list(listen)))

        return z_cfg
