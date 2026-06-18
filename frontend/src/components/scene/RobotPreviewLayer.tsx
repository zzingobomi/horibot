/**
 * Ghost robot preview layer — previewStore 의 선택된 pose 를 반투명 주황 robot 으로.
 *
 * **공통 primitive** (docs/pose_library_design.md §5.1) — calibration 첫 소비자지만
 * calibration 에 가두지 않음. 사용자가 목록에서 클릭한 후보 1개만 표시 (동시에
 * 여러 개 띄우면 헷갈림). MoveIt orange ghost 컨벤션.
 */
import { RobotModel } from "./RobotModel";
import { usePreviewStore } from "@/domain/stores/preview";
import type { RobotInfo } from "@/types/robot";

const GHOST_OPACITY = 0.4;
const GHOST_TINT = "#ff8c1a"; // MoveIt orange

export function RobotPreviewLayer({ robots }: { robots: RobotInfo[] }) {
  const ghosts = usePreviewStore((s) => s.ghosts);
  return (
    <>
      {robots.map((r) => {
        const joints = ghosts[r.id];
        if (!joints) return null;
        return (
          <RobotModel
            key={`ghost-${r.id}`}
            robotType={r.type}
            robotId={r.id}
            basePose={r.base_pose}
            opacity={GHOST_OPACITY}
            tint={GHOST_TINT}
            jointAngles={joints}
            visible
          />
        );
      })}
    </>
  );
}
