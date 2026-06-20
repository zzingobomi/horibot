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
    responses={404: {"description": "Robot not found"}},
)
async def get_calibration_results(robot_id: str):
    """robot 의 calibration 결과를 JSON 으로 반환.

    Intrinsic / Hand-Eye 는 캘 안 됐으면 null 필드. joint_offsets 는 항상 포함
    (없으면 빈 리스트). "캘 안 됨" 은 valid initial state — 200 으로 null 필드
    반환. (이전엔 400 으로 떨어뜨려 frontend useResource 가 영구 fetch-failed
    상태 → useJointOffsetsRad 의 `?? {}` 가 매 render 새 ref → React loop 트리거.)
    """
    try:
        RobotRegistry().get(robot_id)
    except KeyError:
        raise HTTPException(status_code=404, detail=f"robot '{robot_id}' 없음")

    raw = to_json(CalibrationCache().get(robot_id))
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
