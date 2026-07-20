/**
 * 3D scene container — robots 열거 + focus/카메라 계산.
 *
 * robot 상태(joint/TCP frame)는 Robots, 카메라/mesh 는 각 씬 객체 자체가
 * 데이터·대상 robot 을 결정 — 여기엔 "특정 robot 의 stream" 개념이 없다.
 * 남은 책임: robots 목록 / focus / OrbitControls target.
 * (옛 scanRobotId/scanBaseMatrix per-layer plumbing 은 객체 안으로 이동 —
 * [docs/frontend.md].)
 */
import { useCallback, useMemo, useState } from "react";
import { RobotScene } from "./Scene";
import { useRobots } from "@/hooks/useRobots";

interface RobotSceneContainerProps {
  /** focus robot id. null/undefined = 모두 동등 (특정 focus 없음). */
  focusId?: string | null;
}

export function RobotSceneContainer({ focusId }: RobotSceneContainerProps = {}) {
  const { robots } = useRobots();
  const effectiveFocus: string | null = focusId ?? null;

  // focus robot 의 base_pose 로 OrbitControls target.
  const cameraTarget = useMemo<[number, number, number]>(() => {
    if (effectiveFocus !== null) {
      const r = robots.find((x) => x.id === effectiveFocus);
      if (r?.base_pose) {
        return [r.base_pose.x ?? 0, r.base_pose.y ?? 0, (r.base_pose.z ?? 0) + 0.1];
      }
    }
    if (robots.length === 0) return [0, 0.1, 0];
    const acc = robots.reduce(
      (a, r) => {
        a.x += r.base_pose?.x ?? 0;
        a.y += r.base_pose?.y ?? 0;
        a.z += r.base_pose?.z ?? 0;
        return a;
      },
      { x: 0, y: 0, z: 0 },
    );
    const n = robots.length;
    return [acc.x / n, acc.y / n, acc.z / n + 0.1];
  }, [robots, effectiveFocus]);

  // auto-fit — 로드된 focus 로봇 실제 크기(바운딩스피어)로 카메라·그리드 스케일.
  // 로봇마다 크기가 다르므로(so101 0.35m vs UR5e 0.94m) 씬이 크기를 가정하지 않고
  // 로봇에서 파생 (per-robot 원칙). focus 바뀌면 재측정.
  // bounds 에 forFocus 를 달아 focus 불일치 시 무시 (focus 바뀌면 자동으로 stale →
  // reset effect 불필요). handleBounds 가 현재 focus 를 캡처 → focus 변경 시 identity
  // 바뀌어 RobotModel 이 재측정.
  const [bounds, setBounds] = useState<{
    radius: number;
    center: [number, number, number];
    forFocus: string | null;
  } | null>(null);
  const handleBounds = useCallback(
    (radius: number, center: [number, number, number]) =>
      setBounds({ radius, center, forFocus: effectiveFocus }),
    [effectiveFocus],
  );
  const fit = useMemo(() => {
    if (!bounds || bounds.forFocus !== effectiveFocus || bounds.radius <= 0)
      return null;
    return {
      target: bounds.center,
      distance: bounds.radius * 2.6, // 로봇이 화면의 ~1/3 로 잡히는 거리
      gridSize: Math.min(8, Math.max(0.6, bounds.radius * 6)),
    };
  }, [bounds, effectiveFocus]);

  return (
    <RobotScene
      robots={robots}
      focusId={effectiveFocus}
      cameraTarget={fit?.target ?? cameraTarget}
      onBounds={handleBounds}
      fitTarget={fit?.target}
      fitDistance={fit?.distance}
      gridSize={fit?.gridSize}
    />
  );
}
