/**
 * WaypointScenePart — WaypointPanel 의 scenePart (씬 기여 조각, 3D 표현).
 *
 * ghost 미리보기: 선택된 waypoint 의 joint 자세를 반투명 로봇으로 렌더 — "이
 * waypoint 로 MoveJ 하면 어떤 자세가 되나"를 실행 전에 봄 (티치펜던트 정석).
 * joint-space 데이터의 정직한 시각화 — cartesian 파생값 저장/FK 서비스 불필요,
 * RobotModel(URDF) 재사용이라 backend 무변경.
 *
 * scenePart 규약 ([docs/frontend.md]):
 *   - useRobotId() — 패널과 같은 멘탈모델 (RobotProvider 는 ScenePartHost 공급)
 *   - 데이터는 waypointStore (패널 로컬 선택 → store 브리지, scanStore 패턴)
 *   - 패널 닫으면 ghost 도 사라짐 (인스턴스 lifecycle)
 */
import { useRobotId } from "@/hooks/useRobotId";
import { useRobots } from "@/hooks/useRobots";
import { useWaypointStore } from "@/stores/waypointStore";
import { RobotModel } from "@/components/scene/objects/RobotModel";
import { VizColor } from "@/components/scene/theme/visualizationColors";

const GHOST_OPACITY = 0.35;
// PREVIEW(violet) — 가상·예측 표현. "실 로봇 상태가 아니라 이 waypoint 로 가면
// 이렇게 된다"는 의미를 색 체계가 담음 ([scene/theme/visualizationColors]).
const GHOST_TINT = VizColor.PREVIEW;

export function WaypointScenePart() {
  const robotId = useRobotId();
  const { robots } = useRobots();
  const preview = useWaypointStore((s) => s.previews[robotId]);

  const robot = robots.find((r) => r.id === robotId);
  if (!preview || !robot) return null;

  return (
    <RobotModel
      robotType={robot.type}
      basePose={robot.base_pose}
      opacity={GHOST_OPACITY}
      tint={GHOST_TINT}
      jointNames={preview.jointNames}
      jointAngles={preview.jointAngles}
    />
  );
}
