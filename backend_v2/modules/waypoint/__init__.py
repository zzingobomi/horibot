"""Waypoint — Robot Asset Layer (Motion 위).

사람이 티칭한 joint 자세(rad) 를 재사용 자산으로 관리 (Waypoint) + 목적별 묶음
(WaypointGroup). PnP / Scan / Inspection 등 여러 consumer 가 공유. joint 는 rad
저장 (Motion.TcpState 계약 단위) — raw encoder 는 Motion/Driver 내부 구현으로 남김.
설계: docs/task_dsl_waypoint_port.md §4.
"""
