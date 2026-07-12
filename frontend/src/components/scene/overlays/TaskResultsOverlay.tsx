/**
 * TaskResultsOverlay — task 중간값(STEP_RESULT)의 3D 시각화. 기능 오버레이
 * (task 기능 소유, topic 수명 — 결과는 패널이 닫혀도 남는 진단 도구라 scenePart 아님).
 *
 * PICKANDPLACE_STEP_RESULT 를 label 별 누적 (스트림은 latest-only 라 여기서 직접
 * subscribe + 누적), STATE 가 새 run 시작(RUNNING 전이)을 알리면 clear. type 별:
 *   - "list"           → OrientedDetection 모양 원소 회색 소형 sphere (검출 후보들)
 *   - "OrientedDetection" → emerald sphere + 라벨 (선택된 검출 — 라이브 cloud 와
 *     겹쳐 보면 검출 오차가 눈에 보임 = grasp 실패 진단 도구)
 *   - "GraspCandidate" → amber 소형 box (확정 파지점 — ctx.record("grasp"))
 *   - "PlaceCandidate" → amber 소형 box (적치점 — ctx.record("place"))
 * 좌표는 robot base frame → <RobotFrame> 부모 transform (per-robot).
 */
import { useEffect, useMemo, useRef, useState } from "react";
import { Text } from "@react-three/drei";
import { bridge, topicFor } from "@/api/bridge";
import { TaskStatus, Topic } from "@/api/generated/contract";
import type { SceneObjectProps } from "../sceneTypes";
import { RobotFrame } from "../shared/RobotFrame";
import { VizColor } from "../theme/visualizationColors";

interface StepResultMsg {
  label: string;
  type: string;
  value: unknown;
}

export interface Marker {
  key: string;
  kind: "detection" | "candidate" | "position";
  position: [number, number, number];
  label?: string;
}

const isDet = (
  v: unknown,
): v is { position: number[]; prompt: string; score: number } => {
  const d = v as { position?: unknown; prompt?: unknown };
  return Array.isArray(d?.position) && typeof d?.prompt === "string";
};

const isVec3 = (v: unknown): v is [number, number, number] =>
  Array.isArray(v) && v.length === 3 && v.every((n) => typeof n === "number");

/** step 결과 dict → 렌더 marker (순수 — unit test 대상). */
export function extractMarkers(results: Record<string, StepResultMsg>): Marker[] {
  const out: Marker[] = [];
  for (const [label, r] of Object.entries(results)) {
    if (r.type === "OrientedDetection" && isDet(r.value)) {
      const v = r.value;
      out.push({
        key: label,
        kind: "detection",
        position: [v.position[0], v.position[1], v.position[2]],
        label: `${v.prompt} ${(v.score * 100).toFixed(0)}%`,
      });
    } else if (r.type === "list" && Array.isArray(r.value)) {
      r.value.forEach((item, i) => {
        if (isDet(item)) {
          out.push({
            key: `${label}:${i}`,
            kind: "candidate",
            position: [item.position[0], item.position[1], item.position[2]],
          });
        }
      });
    } else if (r.type === "GraspCandidate" || r.type === "PlaceCandidate") {
      // ctx.record 로 노출된 계획 지점 — grasp/place 필드가 목표 TCP 위치.
      const v = r.value as { grasp?: unknown; place?: unknown } | null;
      const pos = r.type === "GraspCandidate" ? v?.grasp : v?.place;
      if (isVec3(pos)) {
        out.push({ key: label, kind: "position", position: pos, label });
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
  const lastStatus = useRef<string>("");

  useEffect(() => {
    const unResult = bridge.subscribe(
      topicFor(Topic.PICKANDPLACE_STEP_RESULT, robotId),
      (d) => {
        const msg = d as unknown as StepResultMsg;
        if (msg.label) setResults((prev) => ({ ...prev, [msg.label]: msg }));
      },
    );
    // 새 run 시작 (비-running → running 전이) = 이전 결과 무효 → clear
    const unState = bridge.subscribe(
      topicFor(Topic.PICKANDPLACE_STATE, robotId),
      (d) => {
        const status = (d as { status?: string }).status ?? "";
        if (status === TaskStatus.RUNNING && lastStatus.current !== TaskStatus.RUNNING) {
          setResults({});
        }
        lastStatus.current = status;
      },
    );
    return () => {
      unResult();
      unState();
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
