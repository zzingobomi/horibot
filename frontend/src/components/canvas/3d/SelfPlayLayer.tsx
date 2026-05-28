import { useSelfPlayStore } from "@/store/selfPlayStore";

/**
 * Self-play attempt 의 3D 마커.
 *
 * - target_xyz: detector 가 본 큐브 위치 (분홍/녹/빨 — stage 결과에 따라)
 * - spike_at_z: descend 중 부딪힌 z 위치 (빨강)
 *
 * store 의 last_result 가 비어있으면 자동 미렌더 → 다른 페이지(Workspace3D
 * 등) 의 RobotScene 에 영향 없음.
 */

const TARGET_COLOR = "#ec4899"; // pink (default)
const SUCCESS_COLOR = "#10b981"; // emerald (s3=OK)
const SPIKE_COLOR = "#ef4444"; // red

export function SelfPlayLayer() {
  const lastResult = useSelfPlayStore((s) => s.state.last_result);
  if (!lastResult || !lastResult.target_xyz) return null;

  const target = lastResult.target_xyz;
  const spikeZ = lastResult.spike_at_z;

  const targetColor =
    lastResult.s3 === "OK"
      ? SUCCESS_COLOR
      : lastResult.s1 === "SPIKE"
        ? SPIKE_COLOR
        : TARGET_COLOR;

  return (
    <>
      {/* target xyz */}
      <mesh position={target}>
        <sphereGeometry args={[0.008, 16, 16]} />
        <meshBasicMaterial color={targetColor} />
      </mesh>

      {/* spike 발생 위치 */}
      {spikeZ !== null && (
        <mesh position={[target[0], target[1], spikeZ]}>
          <sphereGeometry args={[0.006, 12, 12]} />
          <meshBasicMaterial color={SPIKE_COLOR} />
        </mesh>
      )}
    </>
  );
}
