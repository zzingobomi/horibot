/**
 * /robots/:id/* 의 공통 layout — R3F Canvas + 우상단 meta + Outlet.
 *
 * RobotsLayout 은 mode 전환 시 unmount 되지 않음 → R3F (URDF, robot mesh) 는
 * 한 번만 로드되고 mode 갈아탈 때 reuse. 각 mode 컴포넌트 (RobotMoveMode 등)
 * 가 ModeDockview 만 Outlet 으로 렌더 → mode 별 panel set 만 갈아끼움.
 *
 * 옛 frontend RobotsLayout carry over (frontend_v2.md §2.3). backend_v2 의
 * RobotInfo 는 enabled field 박지 X — robots.yaml SSOT (포함된 robot 모두 active).
 */
import { Outlet, useParams } from "react-router-dom";
import { RobotSceneContainer } from "@/components/scene/Container";
import { useRobots } from "@/hooks/useRobots";

export function RobotsLayout() {
  const { id = "" } = useParams<{ id: string }>();
  const { robots, loading, error } = useRobots();

  if (error) {
    return (
      <div className="p-6 text-red-400 font-mono">/robots 응답 실패: {error}</div>
    );
  }
  if (loading) {
    return <div className="p-6 text-zinc-400 font-mono">robots.yaml 로드 중...</div>;
  }

  const found = robots.find((r) => r.id === id);
  if (!found) {
    return (
      <div className="p-6 text-zinc-400 font-mono">
        robot id <span className="text-red-400">{id}</span> 없음. 등록된 robot:{" "}
        {robots.map((r) => r.id).join(", ") || "(없음)"}
      </div>
    );
  }

  return (
    <div
      className="relative w-full h-full overflow-hidden bg-[#080c12]"
      style={{ fontFamily: "'JetBrains Mono', 'Fira Code', monospace" }}
    >
      <div className="absolute inset-0 z-0">
        <RobotSceneContainer focusId={id} />
      </div>

      <Outlet />

      {/* 우상단 meta — mode 전환과 무관하게 robot identity 항상 표시 */}
      <div className="absolute top-3 right-3 z-20 flex items-center gap-2 pointer-events-none">
        <div className="px-3 py-2 rounded bg-zinc-900/80 border border-zinc-700/60 text-zinc-300 text-xs font-mono">
          <div className="text-zinc-100 font-semibold">{found.id}</div>
          <div className="text-zinc-500">type: {found.type}</div>
        </div>
      </div>
    </div>
  );
}
