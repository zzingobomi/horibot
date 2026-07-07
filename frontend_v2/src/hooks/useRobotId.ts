import { useParams } from "react-router-dom";

/**
 * robot-scoped 패널의 대상 robot — `/robots/:id` 라우트 param 이 SSOT.
 *
 * ambient default 로봇 없음: robot 컨텍스트 없이 robot-scoped 패널이 렌더되면
 * 그건 버그 (조용히 기본 로봇으로 라우팅하던 옛 `id ?? DEFAULT_ROBOT_ID` 제거) →
 * 명시적으로 throw 해 잘못된 배치를 즉시 드러낸다. task 패널은 이 훅이 아니라
 * task 바인딩(useTaskRobotId)에서 robot 을 얻는다.
 */
export function useRobotId(): string {
  const { id } = useParams<{ id: string }>();
  if (!id) {
    throw new Error(
      "useRobotId: /robots/:id 라우트 밖에서 robot-scoped 패널이 렌더됨 " +
        "(robot 컨텍스트 필수 — 기본 로봇 fallback 없음)",
    );
  }
  return id;
}
