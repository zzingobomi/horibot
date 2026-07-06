/**
 * 3D scene container — robots 열거 + focus/카메라 계산.
 *
 * robot 상태(joint/TCP frame)는 RobotLayer, 라이브 cloud 는 Scene3DLayer 가
 * robot 마다 자체 구독 — 여기엔 "특정 robot 의 stream" 개념이 없다 (per-robot,
 * N=2 협동 자리). 남은 책임: robots 목록 / focus / OrbitControls target /
 * scan mesh 배치용 focus base matrix.
 */
import { useMemo } from "react";
import { RobotScene } from "./Scene";
import { robotBaseMatrix } from "./transforms";
import { useRobots } from "@/hooks/useRobots";

interface RobotSceneContainerProps {
  /** focus robot id. null = 모두 동등. undefined = default robot. */
  focusId?: string | null;
}

export function RobotSceneContainer({ focusId }: RobotSceneContainerProps = {}) {
  const { robots, defaultId } = useRobots();
  const effectiveFocus: string | null =
    focusId === undefined ? defaultId : focusId;
  // scan(live cloud/mesh) 대상 robot — focus, focus=null(Tasks 등) 은 default.
  const scanRobotId = effectiveFocus ?? defaultId ?? "";

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

  // scan mesh 배치용 — mesh 정점이 robot base frame 이라 base transform 필요.
  const scanBaseMatrix = useMemo(() => {
    const r = robots.find((x) => x.id === scanRobotId);
    return r ? robotBaseMatrix(r.base_pose) : null;
  }, [robots, scanRobotId]);

  return (
    <RobotScene
      robots={robots}
      focusId={effectiveFocus}
      cameraTarget={cameraTarget}
      robotBaseMatrix={scanBaseMatrix}
      robotId={scanRobotId}
    />
  );
}
