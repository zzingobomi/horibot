from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse

from bridge.schemas import CalibrationResults, JointOffsetSchema
from core.coords.joint_coordinates import JointCoordinates
from core.robot.robot_registry import RobotRegistry
from modules.calibration.calibration_cache import CalibrationCache
from modules.calibration.loader import to_json

calibration_router = APIRouter(tags=["calibration"])


@calibration_router.get(
    "/robots/{robot_id}/calibration/results",
    response_model=CalibrationResults,
    responses={
        400: {"description": "Calibration data is not ready"},
        404: {"description": "Robot not found"},
    },
)
async def get_calibration_results(robot_id: str):
    """robot 의 calibration .npz 들을 모아 JSON 으로 반환.

    Hand-Eye / Intrinsic 은 npz 가 없으면 필드 생략. joint_offsets 는 항상 포함
    (없으면 빈 리스트). 분산 모드에서도 PC 가 git 에 있는 같은 파일을 보므로
    프론트엔드는 mount 시 이 엔드포인트 한 번 fetch 로 fresh 한 상태를 받음.

    not-ready (intrinsic & hand_eye 둘 다 누락) 시 400.
    """
    try:
        RobotRegistry().get(robot_id)
    except KeyError:
        raise HTTPException(status_code=404, detail=f"robot '{robot_id}' 없음")

    data = CalibrationCache().get(robot_id)
    if not data.is_ready():
        return JSONResponse(
            content={"error": "Calibration data is not ready"}, status_code=400
        )
    raw = to_json(data)
    return CalibrationResults(
        intrinsic=raw.get("intrinsic"),
        hand_eye=raw.get("hand_eye"),
        joint_offsets=[
            JointOffsetSchema(motor_id=int(mid), offset_rad=float(off))
            for mid, off in sorted(
                JointCoordinates().snapshot(robot_id=robot_id).items()
            )
        ],
    )
