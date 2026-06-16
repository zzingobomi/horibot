"""Pydantic 기반 typed payload schema.

모든 service request/response 와 state 토픽 payload 의 single source of truth.

Frontend 는 bridge 의 /openapi.json 에서 codegen (openapi-typescript) 으로 TS
type 자동 생성 → frontend/src/api/generated/types.ts.

모듈 구성:
- base       — StrictModel / EmptyData / ServiceRequest[T] / ServiceResponse[T]
- motion     — move_j / move_l / move_c / move_p / servo_tcp / speed_tcp / speed_j / get_tcp / stop
- motor      — motor state / cmd schema
- camera     — CameraStatus / DepthFrameHeader (binary 페이로드의 JSON 헤더)
- detector   — detect request / response, detector state
- pointcloud — capture / build mesh / list scans
- calibration — 캘 service request / response
- task       — task tree / state / step result (Step DSL dataclass 호환)
- system     — heartbeat / log message

import 시점에는 가벼움 — 각 sub-module 은 lazy 사용.
"""
