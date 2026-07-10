/**
 * TaskResultsOverlay — task step 결과의 3D 시각화 (v1 포팅). 기능 오버레이
 * (task 기능 소유, topic 수명 — 결과는 패널이 닫혀도 남는 진단 도구라 scenePart 아님).
 *
 * TASK_STEP_RESULT 를 step_id 별 누적 (스트림은 latest-only 라 여기서 직접
 * subscribe + 누적), TASK_TREE 도착(새 preview/run) 시 clear. type 별 dispatch:
 *   - "Detection" → emerald sphere + 라벨 (검출 위치 — 라이브 cloud 와 겹쳐
 *     보면 검출 오차가 눈에 보임 = grasp 실패 진단 도구)
 *   - "list"      → Detection 모양 원소만 회색 소형 sphere (후보들)
 *   - "Position3" → amber 소형 box (grasp/place 목표점)
 * 좌표는 robot base frame → <RobotFrame> 부모 transform (per-robot).
 */
import { useEffect, useMemo, useState } from "react";
import { Text } from "@react-three/drei";
import { bridge, topicFor } from "@/api/bridge";
import { Topic } from "@/api/generated/contract";
import type { SceneObjectProps } from "../sceneTypes";
import { RobotFrame } from "../shared/RobotFrame";
import { VizColor } from "../theme/visualizationColors";

interface StepResultMsg {
  step_id: string;
  type: string;
  value: unknown;
}

export interface Marker {
  key: string;
  kind: "detection" | "candidate" | "position";
  position: [number, number, number];
  label?: string;
}

/** step 결과 dict → 렌더 marker (순수 — unit test 대상). */
export function extractMarkers(results: Record<string, StepResultMsg>): Marker[] {
  const out: Marker[] = [];
  const isDet = (v: unknown): v is { position: number[]; prompt: string; score: number } => {
    const d = v as { position?: unknown; prompt?: unknown };
    return Array.isArray(d?.position) && typeof d?.prompt === "string";
  };
  for (const [stepId, r] of Object.entries(results)) {
    if (r.type === "Detection" && isDet(r.value)) {
      const v = r.value;
      out.push({
        key: stepId,
        kind: "detection",
        position: [v.position[0], v.position[1], v.position[2]],
        label: `${v.prompt} ${(v.score * 100).toFixed(0)}%`,
      });
    } else if (r.type === "list" && Array.isArray(r.value)) {
      r.value.forEach((item, i) => {
        if (isDet(item)) {
          out.push({
            key: `${stepId}:${i}`,
            kind: "candidate",
            position: [item.position[0], item.position[1], item.position[2]],
          });
        }
      });
    } else if (r.type === "Position3") {
      const v = r.value as { x?: number; y?: number; z?: number } | null;
      if (v && typeof v.x === "number") {
        out.push({
          key: stepId,
          kind: "position",
          position: [v.x, v.y ?? 0, v.z ?? 0],
          label: stepId,
        });
      }
    }
  }
  return out;
}

const COLOR: Record<Marker["kind"], string> = {
  detection: VizColor.DETECTION,
  candidate: VizColor.CANDIDATE,
  position: VizColor.TARGET,
};

export function TaskResultsOverlay({ robots, focusId }: SceneObjectProps) {
  // 대상 robot = focus ?? 첫 robot (옛 Container 결정 이동). 로드 전엔 미마운트.
  const robotId = focusId ?? robots[0]?.id ?? "";
  if (!robotId) return null;
  return <ResultsForRobot robotId={robotId} />;
}

function ResultsForRobot({ robotId }: { robotId: string }) {
  const [results, setResults] = useState<Record<string, StepResultMsg>>({});

  useEffect(() => {
    const unResult = bridge.subscribe(
      topicFor(Topic.TASK_STEP_RESULT, robotId),
      (d) => {
        const msg = d as unknown as StepResultMsg;
        if (msg.step_id) setResults((prev) => ({ ...prev, [msg.step_id]: msg }));
      },
    );
    // 새 task/preview 의 tree 도착 = 이전 결과 무효 → clear
    const unTree = bridge.subscribe(topicFor(Topic.TASK_TREE, robotId), () =>
      setResults({}),
    );
    return () => {
      unResult();
      unTree();
    };
  }, [robotId]);

  const markers = useMemo(() => extractMarkers(results), [results]);

  if (markers.length === 0) return null;

  return (
    <RobotFrame robotId={robotId}>
      {markers.map((m) => (
        <group key={m.key} position={m.position}>
          {m.kind === "position" ? (
            <mesh>
              <boxGeometry args={[0.012, 0.012, 0.012]} />
              <meshStandardMaterial color={COLOR[m.kind]} />
            </mesh>
          ) : (
            <mesh>
              <sphereGeometry args={[m.kind === "detection" ? 0.01 : 0.006, 16, 16]} />
              <meshStandardMaterial
                color={COLOR[m.kind]}
                transparent={m.kind === "candidate"}
                opacity={m.kind === "candidate" ? 0.5 : 1.0}
              />
            </mesh>
          )}
          {m.label && (
            <Text
              position={[0, 0, 0.03]}
              fontSize={0.012}
              color={COLOR[m.kind]}
              anchorX="center"
              anchorY="bottom"
              outlineWidth={0.001}
              outlineColor="#000000"
            >
              {m.label}
            </Text>
          )}
        </group>
      ))}
    </RobotFrame>
  );
}
