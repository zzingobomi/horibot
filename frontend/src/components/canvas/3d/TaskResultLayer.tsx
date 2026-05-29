/**
 * TaskResultLayer — task step 출력값을 type 별로 자동 렌더링.
 *
 * Backend 가 `omx/task/step_result` 로 각 step 완료 시 발행하면 taskResultStore
 * 가 누적. 이 layer 가 type 문자열로 dispatch:
 *   - "Detection"  → 노란 sphere (객체 위치) + height bar
 *   - "Position3"  → 청록 marker (policy 출력 / waypoint)
 *   - "Pose6"      → axis triad (orientation 강제 시 — v1 에선 미사용)
 *   - "None"       → 렌더 X (MoveTCP/Gripper/Wait/Home 등 사이드이펙트 step)
 *
 * 새 task tree 가 도착하면 store.clearAll() (useBridge 가 호출) 로 깨끗이.
 * base frame 좌표라 RobotScene 의 z-up→y-up rotation group 안에 마운트되어야 함.
 */
import { useTaskResultStore } from "@/store/taskResultStore";

interface Position3 {
  x: number;
  y: number;
  z: number;
}

interface Detection {
  position: Position3;
  height: number;
  base_z: number;
  confidence: number;
  prompt: string;
}

export function TaskResultLayer() {
  const results = useTaskResultStore((s) => s.results);

  return (
    <group>
      {Object.entries(results).map(([stepId, r]) => {
        if (r.value === null || r.value === undefined) return null;
        if (r.type === "Detection") {
          return (
            <DetectionMarker key={stepId} detection={r.value as Detection} />
          );
        }
        if (r.type === "Position3") {
          return (
            <PositionMarker key={stepId} position={r.value as Position3} />
          );
        }
        return null;
      })}
    </group>
  );
}

function DetectionMarker({ detection }: { detection: Detection }) {
  const { position, height, base_z } = detection;
  // 객체 윗면 sphere + 바닥부터 윗면까지 막대 (height 시각화)
  const barCenterZ = base_z + height / 2;
  return (
    <group>
      <mesh position={[position.x, position.y, position.z]}>
        <sphereGeometry args={[0.008, 16, 16]} />
        <meshStandardMaterial
          color="#ffcc44"
          emissive="#ffaa00"
          emissiveIntensity={0.4}
        />
      </mesh>
      {height > 0.001 && (
        <mesh position={[position.x, position.y, barCenterZ]}>
          <cylinderGeometry args={[0.002, 0.002, height, 8]} />
          <meshStandardMaterial color="#ffcc44" transparent opacity={0.4} />
        </mesh>
      )}
    </group>
  );
}

function PositionMarker({ position }: { position: Position3 }) {
  return (
    <mesh position={[position.x, position.y, position.z]}>
      <sphereGeometry args={[0.005, 12, 12]} />
      <meshStandardMaterial
        color="#44ddff"
        emissive="#44ccff"
        emissiveIntensity={0.6}
      />
    </mesh>
  );
}
