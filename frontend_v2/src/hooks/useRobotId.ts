import { useContext } from "react";
import { RobotContext } from "./robotContext";

/**
 * robot-scoped 패널의 대상 robot — **패널이 소유한 값** ([[robot_ownership_model]]).
 *
 * 옛 구현은 `/robots/:id` route param 을 읽었으나(navigation 에 robot 을 묶는
 * 잘못된 소유권), 이제 robot 은 패널 자기 상태이고 RobotContext 로 공급된다.
 * route 는 패널 **생성 시 초기값** 만 제공하고 이후 관여하지 않는다.
 *
 * ambient default 없음: RobotProvider(=robot 바인딩) 밖에서 robot-scoped 패널이
 * 렌더되면 버그 → 명시적 throw 로 잘못된 배치를 즉시 드러낸다. task 패널은 이
 * 훅이 아니라 task 바인딩(useTaskRobotId)에서 robot 을 얻는다(carve-out).
 */
export function useRobotId(): string {
  const robotId = useContext(RobotContext);
  if (!robotId) {
    throw new Error(
      "useRobotId: RobotProvider 밖에서 robot-scoped 패널이 렌더됨 " +
        "(robot 바인딩 필수 — 환경/기본 로봇 fallback 없음)",
    );
  }
  return robotId;
}
