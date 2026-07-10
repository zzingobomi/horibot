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
import { useEffect, useId, type FunctionComponent, type ReactNode } from "react";
import {
  DockviewDefaultTab,
  type IDockviewPanelProps,
  type IDockviewPanelHeaderProps,
} from "dockview";
import { RobotContext } from "@/hooks/robotContext";
import { useRobots } from "@/hooks/useRobots";
import { describeMissing, missingCapabilities } from "@/lib/capabilities";
import { usePanelInstanceStore } from "@/stores/panelInstanceStore";
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

function UnsupportedEmpty({ reason }: { reason: string }) {
  return (
    <div
      className="h-full w-full flex flex-col items-center justify-center gap-2 p-4 text-center text-zinc-400 font-mono"
      data-testid="capability-unsupported"
    >
      <div className="text-xs">이 robot 에서는 지원하지 않습니다</div>
      {reason && <div className="text-[10px] text-zinc-500">{reason}</div>}
    </div>
  );
}

export interface RobotOwnershipOptions {
  /** 이 패널을 쓰려면 robot 이 가져야 하는 capability (UI 힌트, [[lib/capabilities]]) */
  requiredCapabilities?: readonly string[];
  /** 부족 사유 override (기본은 capability 라벨에서 파생) */
  unavailableReason?: string;
  /**
   * registry PANEL_COMPONENTS key. 이 HOC 는 모든 robot-owned 인스턴스의
   * mount/robotId/capability 를 아는 chokepoint 라, 여기서 panelInstanceStore 에
   * 등록 → Canvas 의 ScenePartHost 가 scene 컴포넌트를 인스턴스별 마운트
   * ([docs/frontend.md]).
   */
  panelKind?: string;
}

/**
 * robot-scoped 패널 HOC. dockview 가 이 컴포넌트를 인스턴스화하며 props.params /
 * props.api 를 준다. 바인딩은 오직 params.robotId — robot 목록을 "바인딩" 에는
 * 쓰지 않는다(§4). 단 capability 판정은 바인딩된 robot 의 사실을 읽어야 하므로
 * useRobots() 로 그 robot 을 찾아 확인한다 (picker 가 목록을 읽는 것과 같은 결 — §6).
 *
 * requiredCapabilities 는 wrap 시점 클로저로 주입(params 아님) → registry 가 유일
 * 소스, 저장 layout 포맷 불변. params.robotId reactive 라 탭 셀렉터로 robot 을
 * 바꾸면 capability 판정도 자동으로 다시 돈다.
 */
export function withRobotOwnership(
  Panel: FunctionComponent,
  options?: RobotOwnershipOptions,
): FunctionComponent<IDockviewPanelProps> {
  const required = options?.requiredCapabilities;
  const reasonOverride = options?.unavailableReason;
  const panelKind = options?.panelKind;

  function RobotOwnedPanel(props: IDockviewPanelProps) {
    const { robots } = useRobots();
    const instanceKey = useId();
    const robotId = readRobotId(props.params);

    // robot 이 목록에 있을 때만 capability 판정 (로딩 전/미상이면 통과 — false
    // "미지원" flash 방지). 백엔드가 어차피 최종 검증하므로 안전.
    const robot = robotId ? robots.find((r) => r.id === robotId) : undefined;
    const missing =
      robot && required && required.length > 0
        ? missingCapabilities(required, robot.capabilities)
        : [];

    // 패널 본문이 실제 렌더되는 조건(바인딩 + capability OK)일 때만 인스턴스 등록
    // — unsupported/미바인딩이면 scenePart 도 자동 미표시 (capability 게이팅 정합).
    const active = !!robotId && missing.length === 0;
    const register = usePanelInstanceStore((s) => s.register);
    const unregister = usePanelInstanceStore((s) => s.unregister);
    useEffect(() => {
      if (!active || !panelKind || !robotId) return;
      register(instanceKey, { panelKind, robotId });
      return () => unregister(instanceKey);
    }, [active, robotId, instanceKey, register, unregister]);

    if (!robotId) {
      return (
        <SelectRobotEmpty
          onSelect={(id) =>
            props.api.updateParameters({ ...props.params, robotId: id })
          }
        />
      );
    }
    if (missing.length > 0) {
      return <UnsupportedEmpty reason={describeMissing(missing, reasonOverride)} />;
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
 * ([docs/frontend.md] §2.3 — 닫아도 다시 추가 가능해야 실수 복구).
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
