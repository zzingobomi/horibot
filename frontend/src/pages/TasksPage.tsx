/**
 * TasksPage — 자연어 pick-and-place 실행 (§17 NL PnP). **최상위 페이지** (/tasks).
 *
 * task/detector/llm 은 host-level(robot-agnostic, backend.md §2.7) — 로봇 하위
 * mode(move/calibrate)가 아니라 최상위 (design: multi_robot_phase2_frontend.md
 * `/tasks/:name` TasksPage, **multi-robot focus=null**).
 *
 * 구조 = RobotsLayout 과 동형이되 focus=null: R3F 씬(RobotSceneContainer focusId={null}
 * — 모든 robot 동등, dim 없음) z-0 + ModeDockview(registry 패널) overlay. floating 패널이
 * 3D 씬 위에 뜨고 dockview 레이아웃이 localStorage 에 영속(§4.1) — RobotsLayout 과 같은
 * 검증된 메커니즘 재사용(SSOT), 별도 shell 안 만듦.
 *
 * 대상 robot 은 task 가 선언 (backend GET /tasks 의 robot_ids) — task 패널이
 * useTaskRobotId 로 조회. ambient default 로봇 없음. 협동(robot_ids 여러 개) task 의
 * 다중 robot UI 는 후속.
 */
import { RobotSceneContainer } from "@/components/scene/Container";
import { ModeDockview, type PanelSpec } from "@/components/shared/ModeDockview";

// title/width/height 는 PANEL_CATALOG(SSOT)에서 derive — 여기선 배치 선언만.
const TASK_PANELS: PanelSpec[] = [
  { id: "prompt", component: "prompt" },
  { id: "task-progress", component: "taskProgress" },
  // 검출 bbox 오버레이 카메라 (v1 tasks 카메라 계승) — 큐브/상자 인식 확인
  { id: "detection-camera", component: "detectionCamera" },
];

export function TasksPage() {
  return (
    <div className="relative h-full w-full overflow-hidden bg-[#080c12]">
      <div className="absolute inset-0 z-0">
        <RobotSceneContainer focusId={null} />
      </div>
      <ModeDockview mode="tasks" panels={TASK_PANELS} />
    </div>
  );
}
