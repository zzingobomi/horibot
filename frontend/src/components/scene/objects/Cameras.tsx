/**
 * Cameras — 카메라 씬 객체 (월드 소유, [docs/frontend.md]).
 *
 * rgbd capability robot 마다 카메라 1대 파생 (robots.yaml SSOT). CameraItem 이
 * 카메라 pose(tcp corrected FK · hand_eye)를 **한 번** 계산하고, 카메라에 딸린
 * 시각 요소를 전부 자기 안에서 그린다:
 *   - frustum + 축  : cameraStore.showFrustum gate (캘/라클 패널이 토글만)
 *   - live cloud    : scanStore.liveEnabled gate (옛 Scene3DLayer 의 검증된
 *                     dynamic buffer/msgpack 로직 이동 — Mirror hand_eye 는 "토글
 *                     시 1회 fetch 가 identity 로 굳던" 사고의 fix, 로직 불변)
 *
 * 패널은 카메라를 그리지 않는다 — 속성(토글)만 제어. frustum 을 원하는 패널이
 * 몇 개든 렌더는 카메라당 한 번 (소유권 모델이 중복을 구조적으로 차단).
 *
 * world 배치 = RobotFrame(base) · cameraInBase(tcp·handEye). cloud 점은 backend
 * 가 camera-frame xyz 로 발행하므로 카메라 group 안 identity 에 놓으면 끝 —
 * 옛 base·tcp·handEye 수동 체인과 동일 결과, pose 수학 중복 제거.
 */
import { useEffect, useMemo } from "react";
import * as THREE from "three";
import { bridge, decodeMsgpackRecord, topicFor } from "@/api/bridge";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type { CalibrationBundle, RobotInfo } from "@/api/generated/contract";
import { useMirror, useStream } from "@/framework";
import { useCameraStore } from "@/stores/cameraStore";
import { useScanStore } from "@/stores/scanStore";
import {
  cameraInBase,
  DEFAULT_FOV,
  fovFromIntrinsic,
  frustumSegmentPositions,
} from "./cameraPose";
import { Frame } from "../shared/primitives";
import { RobotFrame } from "../shared/RobotFrame";
import { VizColor } from "../theme/visualizationColors";
import type { SceneObjectProps } from "../sceneTypes";

const MAX_POINTS = 400_000;
const FRUSTUM_COLOR = VizColor.SENSOR;

export function Cameras({ robots }: SceneObjectProps) {
  return (
    <>
      {/* 카메라 유무 = has_camera (robots.yaml SSOT) — rgbd 로 거르면 USB 웹캠
          robot(omx)의 카메라가 씬에서 통째로 사라짐 (hand_eye 캘 대상인데도).
          rgbd 는 depth 산출물(live cloud)에만 요구. */}
      {robots
        .filter((r) => r.has_camera)
        .map((r) => (
          <CameraItem key={r.id} robot={r} />
        ))}
    </>
  );
}

/** 카메라 1대 — pose 1회 계산 + frustum/cloud 자식 렌더. */
function CameraItem({ robot }: { robot: RobotInfo }) {
  // per-robot 토글 — 패널의 [시야]/[live] 는 자기 robot 카메라만 켠다.
  const showFrustum = useCameraStore((s) => !!s.frustum[robot.id]);
  const rgbd = robot.capabilities?.includes("rgbd") ?? false;
  const liveEnabled = useScanStore((s) => !!s.liveEnabled[robot.id]) && rgbd;

  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId: robot.id });
  // hand_eye — Mirror (snapshot + CALIBRATION_ACTIVATED invalidate+refetch).
  // calibration 은 robot-agnostic — 대상 robot 은 req 필드 (§2.7).
  const bundle = useMirror({
    snapshotService: ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE,
    snapshotReq: { robot_id: robot.id },
    changeTopic: Topic.CALIBRATION_ACTIVATED,
    robotId: robot.id,
  });

  const cam = useMemo(() => {
    if (!tcp.value) return null;
    return cameraInBase(
      tcp.value.position,
      tcp.value.quaternion,
      bundle.value as CalibrationBundle | null,
    );
  }, [tcp.value, bundle.value]);

  // frustum FOV — active intrinsic 이 있으면 실측 시야각, 없으면 D405 스펙 상수.
  const fov = useMemo(
    () =>
      fovFromIntrinsic(bundle.value as CalibrationBundle | null) ?? DEFAULT_FOV,
    [bundle.value],
  );

  const frustumGeom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute(
      "position",
      new THREE.BufferAttribute(frustumSegmentPositions(0.12, fov), 3),
    );
    return g;
  }, [fov]);

  if (!cam || (!showFrustum && !liveEnabled)) return null;

  return (
    <RobotFrame robotId={robot.id}>
      <group
        position={[cam.position[0], cam.position[1], cam.position[2]]}
        quaternion={[
          cam.quaternion[0],
          cam.quaternion[1],
          cam.quaternion[2],
          cam.quaternion[3],
        ]}
      >
        {showFrustum && (
          <>
            <lineSegments>
              <primitive object={frustumGeom} attach="geometry" />
              <lineBasicMaterial color={FRUSTUM_COLOR} transparent opacity={0.7} />
            </lineSegments>
            <Frame
              pose={{ position: [0, 0, 0] }}
              size={0.02}
              label="cam"
              labelColor={FRUSTUM_COLOR}
            />
          </>
        )}
        {liveEnabled && <CameraCloud robotId={robot.id} />}
      </group>
    </RobotFrame>
  );
}

/**
 * live point cloud — camera frame 점을 부모(카메라 group) identity 에 렌더.
 * 마운트 = liveEnabled (부모가 gate) → 구독은 무조건, unmount 가 정리.
 */
function CameraCloud({ robotId }: { robotId: string }) {
  const pointSize = useScanStore((s) => s.pointSize);

  // geometry + dynamic attribute 버퍼 (1회 할당)
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    const pos = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3);
    const col = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3);
    pos.setUsage(THREE.DynamicDrawUsage);
    col.setUsage(THREE.DynamicDrawUsage);
    g.setAttribute("position", pos);
    g.setAttribute("color", col);
    g.setDrawRange(0, 0);
    return g;
  }, []);

  useEffect(() => {
    const wire = topicFor(Topic.SCENE3D_CLOUD, robotId);
    const unsub = bridge.subscribeBinary(wire, (buf) => {
      const rec = decodeMsgpackRecord(buf) as unknown as {
        point_count: number;
        xyz_bytes: Uint8Array;
        rgb_bytes: Uint8Array;
      };
      const n = Math.min(rec.point_count ?? 0, MAX_POINTS);
      if (!n) {
        geom.setDrawRange(0, 0);
        return;
      }
      // msgpack bin view 는 4-byte 정렬 보장 X → 복사 후 Float32 view.
      const xyzU8 = new Uint8Array(rec.xyz_bytes);
      const xyz = new Float32Array(xyzU8.buffer, 0, n * 3);
      const rgb = rec.rgb_bytes;
      const posArr = geom.getAttribute("position").array as Float32Array;
      const colArr = geom.getAttribute("color").array as Float32Array;
      posArr.set(xyz, 0);
      for (let i = 0; i < n * 3; i++) colArr[i] = rgb[i] / 255;
      geom.getAttribute("position").needsUpdate = true;
      geom.getAttribute("color").needsUpdate = true;
      geom.setDrawRange(0, n);
      geom.computeBoundingSphere();
    });
    return unsub;
  }, [robotId, geom]);

  return (
    <points frustumCulled={false}>
      <primitive object={geom} attach="geometry" />
      <pointsMaterial size={pointSize / 1000} vertexColors sizeAttenuation />
    </points>
  );
}
