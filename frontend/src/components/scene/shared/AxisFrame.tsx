import { useMemo, useRef } from "react";
import * as THREE from "three";
import { useFrame } from "@react-three/fiber";
import { Text } from "@react-three/drei";

interface AxisFrameProps {
  matrix?: THREE.Matrix4;
  size?: number;
  label?: string;
  labelColor?: string;
}

function Arrow({
  toward,
  color,
  size,
  axisLabel,
}: {
  toward: THREE.Vector3;
  color: string;
  size: number;
  axisLabel: string;
}) {
  const shaftLen = size * 0.72;
  const coneLen = size * 0.28;
  const shaftR = size * 0.045;
  const coneR = size * 0.1;

  const q = useMemo(
    () =>
      new THREE.Quaternion().setFromUnitVectors(
        new THREE.Vector3(0, 1, 0),
        toward,
      ),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [],
  );

  return (
    <group quaternion={q}>
      <mesh position={[0, shaftLen / 2, 0]}>
        <cylinderGeometry args={[shaftR, shaftR, shaftLen, 10, 1]} />
        <meshStandardMaterial color={color} roughness={0.25} metalness={0.5} />
      </mesh>
      <mesh position={[0, shaftLen + coneLen / 2, 0]}>
        <coneGeometry args={[coneR, coneLen, 10, 1]} />
        <meshStandardMaterial color={color} roughness={0.25} metalness={0.5} />
      </mesh>
      <Text
        position={[0, shaftLen + coneLen + size * 0.15, 0]}
        fontSize={size * 0.32}
        color={color}
        anchorX="center"
        anchorY="middle"
        outlineWidth={size * 0.012}
        outlineColor="#000000"
      >
        {axisLabel}
      </Text>
    </group>
  );
}

export function AxisFrame({
  matrix,
  size = 0.06,
  label,
  labelColor = "#ffffff",
}: AxisFrameProps) {
  const groupRef = useRef<THREE.Group>(null);

  useFrame(() => {
    if (!groupRef.current || !matrix) return;
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const scl = new THREE.Vector3();
    matrix.decompose(pos, quat, scl);
    groupRef.current.position.copy(pos);
    groupRef.current.quaternion.copy(quat);
  });

  return (
    <group ref={groupRef}>
      <mesh>
        <sphereGeometry args={[size * 0.07, 12, 12]} />
        <meshStandardMaterial color="#ffffff" roughness={0.3} metalness={0.4} />
      </mesh>
      <Arrow
        toward={new THREE.Vector3(1, 0, 0)}
        color="#ff3333"
        size={size}
        axisLabel="X"
      />
      <Arrow
        toward={new THREE.Vector3(0, 1, 0)}
        color="#33dd55"
        size={size}
        axisLabel="Y"
      />
      <Arrow
        toward={new THREE.Vector3(0, 0, 1)}
        color="#3399ff"
        size={size}
        axisLabel="Z"
      />
      {label && (
        <Text
          position={[0, -size * 0.35, 0]}
          fontSize={size * 0.28}
          color={labelColor}
          anchorX="center"
          anchorY="top"
          outlineWidth={size * 0.012}
          outlineColor="#000000"
        >
          {label}
        </Text>
      )}
    </group>
  );
}
