from fastapi import APIRouter
from fastapi.responses import JSONResponse

from core.coords.joint_coordinates import JointCoordinates
from modules.calibration.loader import load_calibration, to_json

calibration_router = APIRouter(prefix="/calibration", tags=["calibration"])


@calibration_router.get("/results")
async def get_calibration_results():
    """
    Returns available calibration data as JSON.

    Response shape:
    {
        "intrinsic": { ... },
        "hand_eye":  { ... },
        "joint_offsets": [{"motor_id": int, "offset_rad": float}, ...]
    }

    Hand-Eye/Intrinsic은 .npz가 없으면 필드 생략. joint_offsets는 항상 포함
    (없으면 빈 리스트). 분산 모드에서도 PC가 git에 있는 같은 파일을 보므로
    프론트엔드는 mount 시 이 엔드포인트 한 번 fetch로 fresh한 상태를 받음.
    """
    data = load_calibration()
    if not data.is_ready():
        return JSONResponse(
            content={"error": "Calibration data is not ready"}, status_code=400
        )
    result = to_json(data)
    result["joint_offsets"] = [
        {"motor_id": int(mid), "offset_rad": float(off)}
        for mid, off in sorted(JointCoordinates().snapshot().items())
    ]
    return JSONResponse(content=result)


# TODO: 프론트에서도 해당 api 없애기
# @calibration_router.get("/status")
# async def get_calibration_status():
#     return {
#         "intrinsic": (CALIB_DIR / "intrinsic.npz").exists(),
#         "hand_eye": (CALIB_DIR / "hand_eye.npz").exists(),
#     }
