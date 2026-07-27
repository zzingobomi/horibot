/**
 * HandoverPage — handover task 전용 페이지 (/tasks/handover).
 *
 * PickAndPlacePage 복제 + PANELS/포커스 교체 (task 페이지 = task 별 전용,
 * 2026-07-12 설계 수렴 — 그 파일 docstring 이 레퍼런스). handover 는 robot 2대
 * (giver=omx, receiver=so101) — 씬 포커스는 참여 robot 의 첫 항목(receiver,
 * TASK_ROBOTS 순서 SSOT). 씬에는 두 robot 다 그려진다 (Scene = 월드 소유).
 *
 * 패널: handover(실행 폼 — stop_before_receive 토글 포함) + handoverProgress
 * (진행/디버거) + 검출 오버레이 카메라 (so101 D405 — 공중 재검출 확인).
 */
import { RobotSceneContainer } from "@/components/scene/Container";
import { ModeDockview, type PanelSpec } from "@/components/shared/ModeDockview";
import { useTaskRobots } from "@/hooks/useTaskRobots";
import { ServiceKey } from "@/api/generated/contract";

// title/width/height 는 PANEL_CATALOG(SSOT)에서 derive — 여기선 배치 선언만.
const PANELS: PanelSpec[] = [
  { id: "handover", component: "handover" },
  { id: "handover-progress", component: "handoverProgress" },
  // 검출 bbox/obb 오버레이 카메라 — 봉/상자 인식 확인 (so101 재검출 확인용)
  { id: "detection-camera", component: "detectionCamera" },
];

export function HandoverPage() {
  // 씬 포커스 = task 참여 robot (계약 조회). 로드 전 null = centroid (Container).
  const focusId = useTaskRobots(ServiceKey.HANDOVER_LIST_ROBOTS)[0] ?? null;
  return (
    <div className="relative h-full w-full overflow-hidden bg-[#080c12]">
      <div className="absolute inset-0 z-0">
        <RobotSceneContainer focusId={focusId} />
      </div>
      <ModeDockview mode="handover" panels={PANELS} />
    </div>
  );
}
