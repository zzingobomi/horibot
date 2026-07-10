from __future__ import annotations

import argparse
import asyncio
import logging
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
                entry.name, robots, deploy, runtime, session_factory
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
    from infra.transport.zenoh import ZenohTransport

    deploy, robots = load_configs(host, config_dir)
    transport = ZenohTransport(deploy.zenoh)
    runtime = build_runtime(deploy, robots, transport)

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
    logging.basicConfig(level=logging.INFO)
    asyncio.run(run(args.host))


if __name__ == "__main__":
    main()
