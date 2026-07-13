/**
 * PickAndPlacePage — pick_and_place task 전용 페이지 (/tasks/pick_and_place).
 *
 * 페이지는 task 별 (2026-07-12 설계 수렴): task 마다 UI 요구가 달라서 (핸드오버 =
 * robot 2대 등) 범용 task 페이지 대신 전용 페이지 + 공용 부품 조립. 이 페이지가
 * task 페이지의 레퍼런스 — 새 task 페이지 = 이 파일 복제 + PANELS/포커스 교체.
 *
 * 구조 = RobotsLayout 과 동형: R3F 씬 (RobotSceneContainer — focus 는 task 참여
 * robot) z-0 + ModeDockview (registry 패널) overlay. 대상 robot = 페이지 소유 상수
 * (pickAndPlaceTask.ts — task 정보 채널은 계약뿐, robot 은 패널/페이지가 소유).
 * 협동 task 의 다중 robot UI 는 후속.
 */
import { RobotSceneContainer } from "@/components/scene/Container";
import { ModeDockview, type PanelSpec } from "@/components/shared/ModeDockview";
import { TASK_ROBOT_ID } from "@/pages/pickAndPlaceTask";

// title/width/height 는 PANEL_CATALOG(SSOT)에서 derive — 여기선 배치 선언만.
const PANELS: PanelSpec[] = [
  { id: "pick-and-place", component: "pickAndPlace" },
  { id: "task-progress", component: "taskProgress" },
  // 검출 bbox/obb 오버레이 카메라 — 큐브/상자 인식 확인
  { id: "detection-camera", component: "detectionCamera" },
];

export function PickAndPlacePage() {
  // 씬 포커스 = task 참여 robot (페이지 소유 상수).
  const focusId = TASK_ROBOT_ID;
  return (
    <div className="relative h-full w-full overflow-hidden bg-[#080c12]">
      <div className="absolute inset-0 z-0">
        <RobotSceneContainer focusId={focusId} />
      </div>
      <ModeDockview mode="pick_and_place" panels={PANELS} />
    </div>
  );
}
