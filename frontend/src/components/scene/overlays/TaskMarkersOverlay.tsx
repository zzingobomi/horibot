/**
 * TaskMarkersOverlay — task 가 계산한 계획 산출물(파지/적치 지점)의 3D 시각화.
 * 기능 오버레이 (topic 수명 — 마커는 패널이 닫혀도 남는 진단 도구라 scenePart 아님).
 *
 * 시각화 데이터는 그걸 계산한 쪽이 소유 (detector DETECTIONS 동형): task 가
 * PICKANDPLACE_MARKERS 를 typed 스트림으로 발행, 여기선 그대로 렌더. 발행마다
 * 전체 스냅샷(latest-wins), 새 run 시작(STATE RUNNING 전이)이면 clear.
 * 좌표는 robot base frame → <RobotFrame> 부모 transform (per-robot).
 */
import { useEffect, useRef, useState } from "react";
import { Text } from "@react-three/drei";
import { bridge, topicFor } from "@/api/bridge";
import { TaskStatus, Topic } from "@/api/generated/contract";
import type { TaskMarker, TaskMarkers } from "@/api/generated/contract";
import type { SceneObjectProps } from "../sceneTypes";
import { RobotFrame } from "../shared/RobotFrame";
import { VizColor } from "../theme/visualizationColors";

export function TaskMarkersOverlay({ robots, focusId }: SceneObjectProps) {
  // 대상 robot = focus ?? 첫 robot. 로드 전엔 미마운트.
  const robotId = focusId ?? robots[0]?.id ?? "";
  if (!robotId) return null;
  return <MarkersForRobot robotId={robotId} />;
}

function MarkersForRobot({ robotId }: { robotId: string }) {
  const [markers, setMarkers] = useState<TaskMarker[]>([]);
  const lastStatus = useRef<string>("");

  useEffect(() => {
    const unMarkers = bridge.subscribe(
      topicFor(Topic.PICKANDPLACE_MARKERS, robotId),
      (d) => setMarkers((d as unknown as TaskMarkers).markers ?? []),
    );
    // 새 run 시작 (비-running → running 전이) = 이전 마커 무효 → clear
    const unState = bridge.subscribe(
      topicFor(Topic.PICKANDPLACE_STATE, robotId),
      (d) => {
        const status = (d as { status?: string }).status ?? "";
        if (
          status === TaskStatus.RUNNING &&
          lastStatus.current !== TaskStatus.RUNNING
        ) {
          setMarkers([]);
        }
        lastStatus.current = status;
      },
    );
    return () => {
      unMarkers();
      unState();
    };
  }, [robotId]);

  if (markers.length === 0) return null;

  return (
    <RobotFrame robotId={robotId}>
      {markers.map((m, i) => (
        <group key={`${m.label}:${i}`} position={m.position}>
          <mesh>
            <boxGeometry args={[0.012, 0.012, 0.012]} />
            <meshStandardMaterial color={VizColor.TARGET} />
          </mesh>
          <Text
            position={[0, 0, 0.03]}
            fontSize={0.012}
            color={VizColor.TARGET}
            anchorX="center"
            anchorY="bottom"
            outlineWidth={0.001}
            outlineColor="#000000"
          >
            {m.label}
          </Text>
        </group>
      ))}
    </RobotFrame>
  );
}
