/**
 * Scene primitive — 자주 쓰는 시각 요소의 평범한 R3F 컴포넌트.
 *
 * DSL 아님: <group>/<Center> 옆에 놓이는 그냥 컴포넌트다. 쓰면 편하고 안 쓰면
 * 그만 — scenePart/layer 는 언제든 raw R3F 로 내려갈 수 있다(표현력 천장 없음,
 * [docs/frontend.md]). 렌더 모양은 TaskMarkersOverlay 의
 * marker 렌더(sphere/box/Text)의 일반화.
 *
 * 좌표는 부모 frame 그대로 — robot base frame 에 놓으려면 <RobotFrame> 으로 감쌀 것.
 */
import { useMemo } from "react";
import * as THREE from "three";
import { Line, Text } from "@react-three/drei";
import { AxisFrame } from "./AxisFrame";
import { VizColor } from "../theme/visualizationColors";

export interface Pose {
  position: readonly [number, number, number];
  quaternion?: readonly [number, number, number, number];
}

const IDENTITY_Q = [0, 0, 0, 1] as const;

function q(pose: Pose): readonly [number, number, number, number] {
  return pose.quaternion ?? IDENTITY_Q;
}

function PrimitiveLabel({
  text,
  color,
  offset = 0.03,
}: {
  text: string;
  color: string;
  offset?: number;
}) {
  return (
    <Text
      position={[0, 0, offset]}
      fontSize={0.012}
      color={color}
      anchorX="center"
      anchorY="bottom"
      outlineWidth={0.001}
      outlineColor="#000000"
    >
      {text}
    </Text>
  );
}

/** 좌표축 triad — pose 의 orientation 시각화 (goal pose / board / camera 등). */
export function Frame({
  pose,
  size = 0.04,
  label,
  labelColor = "#ffffff",
}: {
  pose: Pose;
  size?: number;
  label?: string;
  labelColor?: string;
}) {
  return (
    <group
      position={[pose.position[0], pose.position[1], pose.position[2]]}
      quaternion={[q(pose)[0], q(pose)[1], q(pose)[2], q(pose)[3]]}
    >
      <AxisFrame size={size} label={label} labelColor={labelColor} />
    </group>
  );
}

/** 점 marker — sphere + optional label (검출 위치 / waypoint 등). */
export function Marker({
  position,
  color = VizColor.DETECTION,
  radius = 0.01,
  label,
}: {
  position: readonly [number, number, number];
  color?: string;
  radius?: number;
  label?: string;
}) {
  return (
    <group position={[position[0], position[1], position[2]]}>
      <mesh>
        <sphereGeometry args={[radius, 16, 16]} />
        <meshStandardMaterial color={color} />
      </mesh>
      {label && <PrimitiveLabel text={label} color={color} />}
    </group>
  );
}

/** wireframe box — 영역/detection bbox 시각화. extents = 전체 폭 [x,y,z]. */
export function BoxOutline({
  pose,
  extents,
  color = VizColor.TARGET,
  label,
}: {
  pose: Pose;
  extents: readonly [number, number, number];
  color?: string;
  label?: string;
}) {
  const geom = useMemo(
    () =>
      new THREE.EdgesGeometry(
        new THREE.BoxGeometry(extents[0], extents[1], extents[2]),
      ),
    [extents],
  );
  return (
    <group
      position={[pose.position[0], pose.position[1], pose.position[2]]}
      quaternion={[q(pose)[0], q(pose)[1], q(pose)[2], q(pose)[3]]}
    >
      <lineSegments>
        <primitive object={geom} attach="geometry" />
        <lineBasicMaterial color={color} />
      </lineSegments>
      {label && <PrimitiveLabel text={label} color={color} offset={extents[2] / 2 + 0.02} />}
    </group>
  );
}

/** 연결된 폴리라인 — 경로/연결선 시각화. */
export function PolyLine({
  points,
  color = VizColor.SENSOR,
  lineWidth = 1.5,
}: {
  points: readonly (readonly [number, number, number])[];
  color?: string;
  lineWidth?: number;
}) {
  if (points.length < 2) return null;
  return (
    <Line
      points={points as [number, number, number][]}
      color={color}
      lineWidth={lineWidth}
    />
  );
}
