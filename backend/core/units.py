import math

# Dynamixel raw 중심값 (0°)
RAW_CENTER = 2048
RAW_MAX = 4095


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def raw_to_deg(raw: int) -> float:
    return round((raw - RAW_CENTER) / RAW_MAX * 360.0, 2)


def deg_to_raw(deg: float) -> int:
    return int(deg / 360.0 * RAW_MAX + RAW_CENTER)


def raw_to_rad(raw: int, *, reverse: bool = False) -> float:
    angle = (raw - RAW_CENTER) / RAW_MAX * 2.0 * math.pi
    return -angle if reverse else angle


def rad_to_raw(
    radian: float,
    *,
    reverse: bool = False,
    min_raw: int = 0,
    max_raw: int = RAW_MAX,
    offset_rad: float = 0.0,
) -> int:
    """URDF rad → motor raw. offset_rad는 raw_to_rad 가산의 역연산.

    BA가 추정한 motor zero offset이 raw_to_rad(읽기)에 가산되므로, motor 명령
    출력(쓰기) 시에도 차감해야 시스템이 일관됨. 캡처/시각화/명령 모두 같은
    URDF 좌표계로 정렬.
    """
    radian = radian - offset_rad
    if reverse:
        radian = -radian
    raw = int(radian / (2.0 * math.pi) * RAW_MAX + RAW_CENTER)
    return max(min_raw, min(max_raw, raw))
