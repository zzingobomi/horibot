/**
 * ScenePartHost — 살아있는 패널 인스턴스의 scenePart 를 Canvas 에 마운트.
 *
 * panelInstanceStore(withRobotOwnership 이 등록) × PANEL_CATALOG.scenePart(registry
 * 선언) 의 교집합을 인스턴스별로 렌더. 각 조각을 <RobotProvider> 로 감싸므로
 * scenePart 안은 패널 코드와 같은 멘탈모델(useRobotId/useStream 그대로).
 *
 * 경계를 runtime 에 넘는 것은 인스턴스 목록(데이터)뿐 — scene 컴포넌트 자체는
 * registry 에 정적 등록 (identity 안정, runtime JSX 주입 기각 근거는
 * [docs/frontend.md]). 같은 패널 2개(robot A/B)면 조각도
 * 2개 — 각자 자기 robot 으로. Scene.tsx 는 이 컴포넌트 한 줄만 안다.
 */
import { PANEL_CATALOG, type PanelComponentKey } from "@/components/panels/registry";
import { RobotProvider } from "@/components/shared/robotOwnership";
import { usePanelInstanceStore } from "@/stores/panelInstanceStore";

export function ScenePartHost() {
  const instances = usePanelInstanceStore((s) => s.instances);

  return (
    <>
      {Object.entries(instances).map(([key, inst]) => {
        const ScenePart = PANEL_CATALOG[inst.panelKind as PanelComponentKey]?.scenePart;
        if (!ScenePart) return null;
        return (
          <RobotProvider key={key} robotId={inst.robotId}>
            <ScenePart />
          </RobotProvider>
        );
      })}
    </>
  );
}
