/**
 * <RobotFrame> — "robot base frame 용 group" ([docs/frontend.md]).
 *
 * 자식은 robot base frame(백엔드 좌표 그대로, z-up)에서 그려진다 — z-up→y-up +
 * base_pose 변환은 여기가 담당(transforms.ts SSOT). R3F 에서 transform 은 원래
 * 트리에서 명시하는 것(<group position>)이고, 이것은 그 문법의 robot 좌표계 버전일
 * 뿐 — DSL 아님. 좌표계 선택은 렌더링 내용의 일부이므로 프레임워크가 자동 래핑으로
 * 숨기지 않는다(auto-wrap 기각) — mixed-frame(robot 기하 + world 기하 형제 배치)이
 * 자유로워야 하기 때문.
 *
 * robotId 생략 = RobotContext 의 자기 robot (ScenePartHost/RobotProvider 가 공급 —
 * 패널의 useRobotId 와 같은 계약: context 도 명시도 없으면 배선 버그이므로 throw).
 * robot 이 목록에 아직 없으면(로딩) null — transient 데이터 상태는 조용히 대기.
 */
import { useContext, useMemo, type ReactNode } from "react";
import * as THREE from "three";
import { RobotContext } from "@/hooks/robotContext";
import { useRobots } from "@/hooks/useRobots";
import { robotBaseMatrix } from "./transforms";

interface RobotFrameProps {
  /** 대상 robot. 생략 시 RobotContext (scenePart 안이면 자기 패널의 robot). */
  robotId?: string;
  children: ReactNode;
}

export function RobotFrame({ robotId, children }: RobotFrameProps) {
  const ctxRobotId = useContext(RobotContext);
  const target = robotId ?? ctxRobotId;
  const { robots } = useRobots();

  if (!target) {
    throw new Error(
      "RobotFrame: robotId 명시도 RobotContext 도 없음 — scenePart/RobotProvider " +
        "밖에서 쓰려면 robotId 를 명시할 것",
    );
  }

  const frame = useMemo(() => {
    const r = robots.find((x) => x.id === target);
    if (!r) return null;
    const m = robotBaseMatrix(r.base_pose);
    const p = new THREE.Vector3();
    const q = new THREE.Quaternion();
    const s = new THREE.Vector3();
    m.decompose(p, q, s);
    return {
      position: [p.x, p.y, p.z] as const,
      quaternion: [q.x, q.y, q.z, q.w] as const,
    };
  }, [robots, target]);

  if (!frame) return null; // robot 목록 로딩 전 — transient

  return (
    <group
      position={[frame.position[0], frame.position[1], frame.position[2]]}
      quaternion={[
        frame.quaternion[0],
        frame.quaternion[1],
        frame.quaternion[2],
        frame.quaternion[3],
      ]}
    >
      {children}
    </group>
  );
}
