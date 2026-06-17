/**
 * /robots/:id 인덱스 라우트 — capabilities[0] 모드로 redirect.
 *
 * Sidebar 의 collapsed 모드에서는 robot 1 개당 1 아이콘만 보여서 클릭 시
 * /robots/:id (mode 없음) 로 진입함 — 이 자리에서 사용 가능한 첫 모드로
 * replace navigation. 사용 가능한 mode 가 0개면 안내 화면 표시.
 */
import { Navigate, useParams } from "react-router-dom";
import { useRobots } from "@/hooks/useRobots";

export function RobotModeRedirect() {
  const { id = "" } = useParams<{ id: string }>();
  const { robots, loading } = useRobots();

  if (loading) return null; // RobotsLayout 가 동일 hook 으로 로딩 화면 표시 중

  const robot = robots.find((r) => r.id === id);
  // gamepad / rgbd 같은 도구성/sensor capability 는 sidebar mode sub-route 없음 —
  // UI mode 자리 (move/calibrate) 만 redirect 대상. point cloud 자리는 Scene
  // Controls 토글 (mode 무관). scan workflow 자리는 TasksPage 의 scan task.
  const PAGE_MODES = new Set(["move", "calibrate"]);
  const firstMode = robot?.capabilities.find((c) => PAGE_MODES.has(c));
  if (firstMode) {
    return <Navigate to={`/robots/${id}/${firstMode}`} replace />;
  }
  return (
    <div className="absolute inset-0 z-10 flex items-center justify-center pointer-events-none">
      <div className="px-4 py-3 rounded bg-zinc-900/80 border border-zinc-700/60 text-zinc-400 text-xs font-mono">
        {robot
          ? `${robot.id} 에 활성화된 모드 없음 — robots.yaml::capabilities 확인`
          : `robot ${id} 등록 안 됨`}
      </div>
    </div>
  );
}
