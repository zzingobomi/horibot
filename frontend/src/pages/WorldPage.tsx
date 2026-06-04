/**
 * /world — overview mode. multi_robot_phase2_frontend.md §2 sketch:
 * WorldScene (free orbit, world_overview camera), 모든 robot 동등하게 보임.
 *
 * 첫 프로토타입 scope — Panel 없음 / Layer toggle bar 없음. focus=null 만.
 */
import { RobotSceneContainer } from "@/components/canvas/RobotSceneContainer";

export function WorldPage() {
  return (
    <div
      className="relative w-full h-full overflow-hidden bg-[#080c12]"
      style={{ fontFamily: "'JetBrains Mono', 'Fira Code', monospace" }}
    >
      <div className="absolute inset-0 z-0">
        <RobotSceneContainer focusId={null} />
      </div>
      <div className="absolute top-3 right-3 z-20 px-3 py-2 rounded bg-zinc-900/80 border border-zinc-700/60 text-zinc-400 text-xs font-mono">
        World overview
      </div>
    </div>
  );
}
