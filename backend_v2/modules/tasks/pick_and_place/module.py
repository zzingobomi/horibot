import logging

from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.detector.contract import Detector, DetectRequest, DetectResponse


from .contract import PickAndPlace, RunRequest, RunResponse

logger = logging.getLogger(__name__)


class PickAndPlaceModule:
    def __init__(self, runtime: ModuleRuntime) -> None:
        self.runtime = runtime

    async def start(self) -> None:
        logger.info("PickAndPlaceModule start")

    async def stop(self) -> None:
        logger.info("PickAndPlaceModule stop")

    @service(PickAndPlace.Service.RUN)
    async def run(self, req: RunRequest) -> RunResponse:

        # detect 모듈에게 결과 받기
        result = await self.runtime.call(
            Detector.Service.DETECT,
            DetectRequest(robot_id="so101_6dof_0", prompt="white box", top_k=5),
            DetectResponse,
        )
        logger.info("found=%s n=%d", result.found, len(result.candidates))

        # grab 전략 세우기

        return RunResponse()
