import { createContext } from "react";

/**
 * robot-scoped 패널의 대상 robot 을 하위 트리에 공급하는 context.
 *
 * robot 소유권 모델(docs/frontend.md)의 배선 지점: robot 은 패널이
 * 소유(패널 자기 상태)하고, 그 값을 이 context 로 패널 내부에 흘려보낸다. 패널
 * 내부 코드는 useRobotId() 로 이 값을 읽을 뿐 — route/robot 목록 같은 환경을
 * 직접 읽지 않는다(§4 바인딩 불변식).
 *
 * value=null 은 "아직 robot 미바인딩" — withRobotOwnership 가 Provider 를 아예
 * 씌우지 않고 Select Robot 빈 상태를 대신 렌더하므로, 정상 경로에서 패널 내부가
 * null context 를 보는 일은 없다(useRobotId 는 그때 throw = 잘못된 배치 표면화).
 */
export const RobotContext = createContext<string | null>(null);
