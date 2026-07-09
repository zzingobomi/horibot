/**
 * robot 소유권 모델([[robot_ownership_model]])의 프론트 배선.
 *
 * - RobotProvider: 패널이 소유한 robot 을 하위 트리에 공급 (useRobotId 가 읽음).
 * - withRobotOwnership: robot-scoped 패널 컴포넌트를 감싸, params.robotId 를 읽어
 *   (a) 미바인딩이면 Select Robot 빈 상태, (b) 바인딩되면 Provider 로 감싸 렌더.
 * - RobotTab: 패널 탭. 제목 + (robot 2개 이상일 때만) robot 셀렉터.
 *
 * "환경을 읽지 않는다"(§4)는 **바인딩** 규칙 — 패널이 어느 robot 데이터를 보여줄지는
 * 오직 params.robotId 에서 온다. 셀렉터가 목록을 읽고 "N=1 이면 숨김" 이 개수를 읽는
 * 것은 picker/cosmetic 이라 위반이 아니다(§6).
 */
import type { FunctionComponent, ReactNode } from "react";
import {
  DockviewDefaultTab,
  type IDockviewPanelProps,
  type IDockviewPanelHeaderProps,
} from "dockview";
import { RobotContext } from "@/hooks/robotContext";
import { useRobots } from "@/hooks/useRobots";
import { RobotSelect } from "./RobotSelect";

export function RobotProvider({
  robotId,
  children,
}: {
  robotId: string;
  children: ReactNode;
}) {
  return <RobotContext.Provider value={robotId}>{children}</RobotContext.Provider>;
}

function readRobotId(params: Record<string, unknown> | undefined): string | null {
  const v = params?.robotId;
  return typeof v === "string" && v.length > 0 ? v : null;
}

function SelectRobotEmpty({ onSelect }: { onSelect: (robotId: string) => void }) {
  return (
    <div className="h-full w-full flex flex-col items-center justify-center gap-3 text-zinc-400 font-mono">
      <div className="text-xs">이 패널의 대상 robot 을 선택하세요</div>
      <RobotSelect value={null} onChange={onSelect} className="text-xs px-2 py-1" />
    </div>
  );
}

/**
 * robot-scoped 패널 HOC. dockview 가 이 컴포넌트를 인스턴스화하며 props.params /
 * props.api 를 준다. 바인딩은 오직 params.robotId — robot 목록을 참조하지 않는다.
 */
export function withRobotOwnership(
  Panel: FunctionComponent,
): FunctionComponent<IDockviewPanelProps> {
  function RobotOwnedPanel(props: IDockviewPanelProps) {
    const robotId = readRobotId(props.params);
    if (!robotId) {
      return (
        <SelectRobotEmpty
          onSelect={(id) =>
            props.api.updateParameters({ ...props.params, robotId: id })
          }
        />
      );
    }
    return (
      <RobotProvider robotId={robotId}>
        <Panel />
      </RobotProvider>
    );
  }
  RobotOwnedPanel.displayName = `RobotOwned(${Panel.displayName || Panel.name || "Panel"})`;
  return RobotOwnedPanel;
}

/**
 * robot-scoped 패널의 탭 — 제목 + robot 셀렉터 + 닫기.
 * robot 이 하나뿐이면 셀렉터를 숨긴다(선택지 하나짜리 picker = 노이즈, §6).
 * 닫기는 auto-hide 헤더의 `+ 패널 추가` 와 세트라 활성
 * ([docs/workspace_autohide_header.md] §2.3 — 닫아도 다시 추가 가능해야 실수 복구).
 */
export function RobotTab(props: IDockviewPanelHeaderProps) {
  const { robots } = useRobots();
  const robotId = readRobotId(props.params);
  const showSelector = robots.length > 1;

  return (
    <div className="flex items-center gap-1.5 pr-1">
      <DockviewDefaultTab {...props} />
      {showSelector && (
        <RobotSelect
          value={robotId}
          onChange={(id) =>
            props.api.updateParameters({ ...props.params, robotId: id })
          }
        />
      )}
    </div>
  );
}
