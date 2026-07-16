from __future__ import annotations

import argparse
import asyncio
import logging
import os
import signal
from pathlib import Path

from framework.runtime.app import Runtime
from framework.transport.protocol import Transport

from .config import DeploymentConfig, RobotConfig, load_deployment, load_robots
from .registry import load_module_class
from .resolve import resolve_robot_deps, resolve_host_deps

logger = logging.getLogger(__name__)


_CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"


def build_runtime(
    deploy: DeploymentConfig,
    robots: dict[str, RobotConfig],
    transport: Transport,
    *,
    host: str = "",
) -> Runtime:
    runtime = Runtime(transport)

    session_factory = None
    if deploy.rdb_uri:
        from infra.database.boot import open_database, run_migrations

        engine, session_factory = open_database(deploy.rdb_uri)
        # TODO: 여러 프로세스가 동일한 Postgres에 대해 migration을 수행할 수 있다면
        # upgrade race를 방지하기 위해 advisory lock으로 직렬화해야 한다.
        run_migrations(engine)
        logger.info(
            "DB ready — rdb_uri=%s (engine + session_factory + alembic upgrade head)",
            deploy.rdb_uri,
        )

    for entry in deploy.modules:
        mod_cls = load_module_class(entry.name)
        if entry.robots:
            for rid in entry.robots:
                robot = robots.get(rid)
                if robot is None:
                    raise KeyError(
                        f"module {entry.name} 의 robot {rid!r} 가 robots.yaml 에 없음"
                    )
                if not robot.enabled:
                    # robots.yaml spec — enabled=false 는 런타임이 무시.
                    logger.warning(
                        "module %s 의 robot %s 는 enabled=false — skip", entry.name, rid
                    )
                    continue
                deps = resolve_robot_deps(entry.name, robot, deploy, session_factory)
                runtime.add_module(mod_cls, robot_id=rid, **deps)
                logger.info("add_module %s robot_id=%s", entry.name, rid)
        else:
            deps = resolve_host_deps(
                entry.name, robots, deploy, runtime, session_factory, host=host
            )
            runtime.add_module(mod_cls, **deps)
            logger.info("add_module %s (host-level)", entry.name)

    return runtime


def load_configs(
    host: str, config_dir: Path = _CONFIG_DIR
) -> tuple[DeploymentConfig, dict[str, RobotConfig]]:
    deploy = load_deployment(config_dir / "deployments" / f"{host}.yaml")
    robots = load_robots()
    return deploy, robots


async def run(host: str, config_dir: Path = _CONFIG_DIR) -> None:
    from infra.logging.publisher import attach_log_publisher, detach_log_publisher
    from infra.transport.zenoh import ZenohTransport

    deploy, robots = load_configs(host, config_dir)
    transport = ZenohTransport(deploy.zenoh)
    # transport 준비 직후 발행 핸들러 부착 — 이 host 의 모든 모듈 로그가 콘솔과
    # 함께 log/{host} 로도 나간다 (docs/logging.md §1). 중앙 PC 의 logcollector 가 수집.
    log_publisher = attach_log_publisher(transport, host)
    runtime = build_runtime(deploy, robots, transport, host=host)

    await runtime.start()
    logger.info("Runtime started — host=%s modules=%d", host, len(deploy.modules))

    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    try:
        await stop_event.wait()
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        logger.info("Runtime stopping — host=%s", host)
        await runtime.stop()
        # transport.close 전에 발행 핸들러 제거 — 닫힌 세션으로 발행 시도 방지.
        # (runtime.stop 까지의 shutdown 로그는 아직 발행됨.)
        detach_log_publisher(log_publisher)
        transport.close()
        logger.info("Runtime stopped — host=%s", host)


def main() -> None:
    parser = argparse.ArgumentParser(description="horibot backend boot")
    parser.add_argument(
        "--host",
        required=True,
        help="deployment yaml 이름 (mock / pc / pi_hori1 / ...)",
    )
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s.%(msecs)03d [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    # 특정 로거만 DEBUG (전역 DEBUG 는 zenoh 등 홍수) — 실물 디버깅용.
    # 예: HORIBOT_DEBUG_LOGGERS=modules.motion.adapters.pybullet → IK walk 상세
    for name in os.environ.get("HORIBOT_DEBUG_LOGGERS", "").split(","):
        if name.strip():
            logging.getLogger(name.strip()).setLevel(logging.DEBUG)
    asyncio.run(run(args.host))


if __name__ == "__main__":
    main()
