/**
 * HandoverPage — handover task 전용 페이지 (/tasks/handover).
 *
 * PickAndPlacePage 복제 + PANELS/포커스 교체 (task 페이지 = task 별 전용,
 * 2026-07-12 설계 수렴 — 그 파일 docstring 이 레퍼런스). handover 는 robot 2대
 * (giver=omx, receiver=so101)가 **둘 다 주역** — 씬 포커스 없음 (focusId=null =
 * 모두 동등 + centroid 프레이밍. 2026-07-27 실물: receiver 포커스를 넘기니
 * 참여 robot 인 omx 가 딤 처리되는 오표시).
 *
 * 패널: handover(실행 폼 — stop_before_receive 토글 포함) + handoverProgress
 * (진행/디버거) + 검출 오버레이 카메라 (robot 셀렉터로 omx 관측/so101 재검출
 * 각각 확인 — 필요하면 패널 추가로 둘 다 띄움).
 */
import { RobotSceneContainer } from "@/components/scene/Container";
import { ModeDockview, type PanelSpec } from "@/components/shared/ModeDockview";

// title/width/height 는 PANEL_CATALOG(SSOT)에서 derive — 여기선 배치 선언만.
const PANELS: PanelSpec[] = [
  { id: "handover", component: "handover" },
  { id: "handover-progress", component: "handoverProgress" },
  // 검출 bbox/obb 오버레이 카메라 — 봉/상자 인식 확인 (omx 관측 + so101 재검출)
  { id: "detection-camera", component: "detectionCamera" },
];

export function HandoverPage() {
  return (
    <div className="relative h-full w-full overflow-hidden bg-[#080c12]">
      <div className="absolute inset-0 z-0">
        <RobotSceneContainer focusId={null} />
      </div>
      <ModeDockview mode="handover" panels={PANELS} />
    </div>
  );
}
