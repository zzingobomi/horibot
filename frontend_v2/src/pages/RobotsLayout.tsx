/**
 * /robots/:id/* 의 공통 layout — R3F Canvas + Outlet.
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
      {/* 우상단 robot id/type 박스는 제거 — 사이드바와 순수 중복
          ([docs/workspace_autohide_header.md] §2.2) */}
    </div>
  );
}
