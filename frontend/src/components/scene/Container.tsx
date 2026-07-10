/**
 * 3D scene container — robots 열거 + focus/카메라 계산.
 *
 * robot 상태(joint/TCP frame)는 Robots, 카메라/mesh 는 각 씬 객체 자체가
 * 데이터·대상 robot 을 결정 — 여기엔 "특정 robot 의 stream" 개념이 없다.
 * 남은 책임: robots 목록 / focus / OrbitControls target.
 * (옛 scanRobotId/scanBaseMatrix per-layer plumbing 은 객체 안으로 이동 —
 * [docs/frontend.md].)
 */
import { useMemo } from "react";
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

  return (
    <RobotScene
      robots={robots}
      focusId={effectiveFocus}
      cameraTarget={cameraTarget}
    />
  );
}
