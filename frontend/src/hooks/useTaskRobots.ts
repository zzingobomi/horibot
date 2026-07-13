/**
 * useTaskRobots — task 참여 robot 명부 (task 공용).
 *
 * SSOT = backend task 모듈의 참여 robot 상수(예: pick_and_place TASK_ROBOTS),
 * 채널 = 각 task 계약의 list_robots 서비스 (task 정보 채널은 계약뿐). task
 * 패널은 robot 을 *고르지* 않고 task 가 *알려주는* 이 목록으로 robot-scoped
 * 스트림 키 `{robot_id}` 를 채운다 — 옛 하드코딩 상수(TASK_ROBOT_ID) 대체.
 *
 * 새 task = 자기 계약의 list_robots 키를 넘기면 끝:
 *   const robots = useTaskRobots(ServiceKey.PICKANDPLACE_LIST_ROBOTS);
 *
 * useService 캐시 = (key, robotId 없음) 전역 공유 — 여러 컴포넌트가 같이 써도
 * fetch 는 사실상 1회. 로드 전엔 빈 배열 (scoped 키 미확장 fail-soft — 데이터
 * 없음일 뿐 엉뚱한 robot 으로 새지 않음).
 */
import { useEffect } from "react";
import { useBridgeConnected, useService } from "@/framework";
import type { ServiceMap } from "@/api/generated/contract";

/** res 가 robot 명부 shape 인 서비스 키만 허용 — 엉뚱한 키는 타입 에러. */
type TaskRobotsServiceKey = {
  [K in keyof ServiceMap]: ServiceMap[K]["res"] extends { robot_ids: string[] }
    ? K
    : never;
}[keyof ServiceMap];

export function useTaskRobots(key: TaskRobotsServiceKey): string[] {
  const connected = useBridgeConnected();
  const { call, data, pending } = useService(key);

  useEffect(() => {
    // WS 미연결 시 callService 가 drop → connected 를 dep 으로 연결 후 fetch
    // (WaypointPanel 초기 목록 로드와 동일 패턴). 캐시에 있으면 재호출 없음.
    if (!connected || data || pending) return;
    void call({});
  }, [connected, data, pending, call]);

  return data?.robot_ids ?? [];
}
