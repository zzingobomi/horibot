from enum import StrEnum

from framework.contract.model import DraftModel


class PickAndPlace:
    class Service(StrEnum):
        RUN = "srv/pick_and_place/run"


class RunRequest(DraftModel):
    pass


class RunResponse(DraftModel):
    pass
