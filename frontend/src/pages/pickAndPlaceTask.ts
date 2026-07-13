/**
 * pick_and_place task 전용 페이지의 SSOT 상수 (2026-07-13).
 *
 * task 의 정보 채널은 계약뿐 (GET /tasks 폐지) — robot 바인딩/표시 문구는 task
 * 전용 페이지가 소유한다 ("robot 은 패널이 소유" 원칙). ROBOT_ID 는 backend
 * scenario 의 `ctx.robot("so101_6dof_0")` 리터럴과 같은 값 — task 가 어느 robot
 * 을 움직이는지는 task 코드의 사실이고, 페이지는 그 사실을 자기 상수로 가진다.
 */

export const TASK_NAME = "pick_and_place";

/** task 참여 robot — 스트림 키 `{robot_id}` 채움 + 씬 포커스. */
export const TASK_ROBOT_ID = "so101_6dof_0";
