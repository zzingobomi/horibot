/**
 * /robots/:id 인덱스 라우트 — 사용 가능한 첫 mode 로 redirect.
 *
 * Sidebar collapsed 모드에서 robot 아이콘 클릭 시 /robots/:id (mode 없음) 진입 —
 * 이 자리에서 사용 가능한 첫 모드로 replace navigation.
 *
 * PAGE_MODES = UI page mode 만 (현재 move — calibrate 는 Step E+). gamepad / rgbd
 * 같은 도구성/sensor capability 는 sidebar mode sub-route 아님. capabilities 가
 * 비어 있어도 arm robot 은 move 가능 → move fallback.
 */
import { Navigate, useParams } from "react-router-dom";
import { useRobots } from "@/hooks/useRobots";

const PAGE_MODES: ReadonlySet<string> = new Set(["move"]);

export function RobotModeRedirect() {
  const { id = "" } = useParams<{ id: string }>();
  const { robots, loading } = useRobots();

  if (loading) return null; // RobotsLayout 가 동일 hook 으로 로딩 화면 표시 중

  const robot = robots.find((r) => r.id === id);
  if (!robot) {
    return (
      <div className="absolute inset-0 z-10 flex items-center justify-center pointer-events-none">
        <div className="px-4 py-3 rounded bg-zinc-900/80 border border-zinc-700/60 text-zinc-400 text-xs font-mono">
          robot {id} 등록 안 됨
        </div>
      </div>
    );
  }

  const caps = robot.capabilities ?? [];
  const firstMode = caps.find((c) => PAGE_MODES.has(c)) ?? "move";
  return <Navigate to={`/robots/${id}/${firstMode}`} replace />;
}
