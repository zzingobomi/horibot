/**
 * TasksPage — 자연어 pick-and-place 실행 (§17 NL PnP). **최상위 페이지** (/tasks).
 *
 * task/detector/llm 은 host-level(robot-agnostic, backend_v2.md §2.7) — 로봇 하위
 * mode(move/calibrate)가 아니라 최상위 (design: multi_robot_phase2_frontend.md
 * `/tasks/:name` TasksPage, **multi-robot focus=null**).
 *
 * 구조 = RobotsLayout 과 동형이되 focus=null: R3F 씬(RobotSceneContainer focusId={null}
 * — 모든 robot 동등, dim 없음) z-0 + ModeDockview(registry 패널) overlay. floating 패널이
 * 3D 씬 위에 뜨고 dockview 레이아웃이 localStorage 에 영속(§4.1) — RobotsLayout 과 같은
 * 검증된 메커니즘 재사용(SSOT), 별도 shell 안 만듦.
 *
 * 현재 N=1 → 패널이 useParams id 부재 시 DEFAULT_ROBOT_ID. multi-robot 로봇 선택 UI +
 * TaskResultLayer(검출 3D)는 후속 (frontend_v2.md §15 Step E+) — 이 씬에 slot-in.
 */
import { RobotSceneContainer } from "@/components/scene/Container";
import { ModeDockview, type PanelSpec } from "@/components/shared/ModeDockview";

const TASK_PANELS: PanelSpec[] = [
  { id: "prompt", component: "prompt", title: "Command", width: 340, height: 260 },
  {
    id: "task-progress",
    component: "taskProgress",
    title: "Task Progress",
    width: 340,
    height: 420,
  },
  // 검출 bbox 오버레이 카메라 (v1 tasks 카메라 계승) — 큐브/상자 인식 확인
  {
    id: "detection-camera",
    component: "detectionCamera",
    title: "Camera (Detection)",
    width: 440,
    height: 330,
  },
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
