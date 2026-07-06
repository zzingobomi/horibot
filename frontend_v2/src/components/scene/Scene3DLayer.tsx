/**
 * Scene3DLayer — 라이브 point cloud (backend scene3d module). robotId 로 자립.
 *
 * scene3d 가 camera-frame xyz+rgb 발행 (Scene3dCloud, msgpack bin) → 여기서
 * 부모 transform = base · tcp · hand_eye 로 world 배치. 필요한 상태를 전부
 * 자체 구독 (per-robot — N robot 시 robotId 만 다르게 하나 더 마운트):
 *   - TCP pose  : useStream(MOTION_TCP_STATE, robotId) — backend corrected FK SSOT
 *   - hand_eye  : useMirror(CALIBRATION_SNAPSHOT_BUNDLE + CALIBRATION_ACTIVATED)
 *                 — mount/재연결/캘 activate 시 자동 refetch. (옛 "토글 시 1회
 *                 fetch" 는 타임아웃 시 identity 로 조용히 굳어 cloud 가 공중에
 *                 뜨던 원인 — Mirror 로 제거.)
 *   - pointSize : scanStore (LivePointCloud 패널의 시각 옵션)
 *
 * liveEnabled(scanStore) gate — [라이브] 켤 때만 cloud 구독.
 */
import { useEffect, useMemo } from "react";
import * as THREE from "three";
import { bridge, decodeMsgpackRecord, topicFor } from "@/api/bridge";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type { CalibrationBundle } from "@/api/generated/contract";
import { useMirror, useStream } from "@/framework";
import { useRobots } from "@/hooks/useRobots";
import { useScanStore } from "@/stores/scanStore";
import { robotBaseMatrix, poseToWorldMatrix } from "./transforms";

const MAX_POINTS = 400_000;

function handEyeMatrix(bundle: CalibrationBundle | null): THREE.Matrix4 {
  const he = bundle?.hand_eye?.result_data;
  if (!he) return new THREE.Matrix4(); // identity fallback (캘 전 / mock)
  const r = he.R_cam2gripper;
  const t = he.t_cam2gripper;
  const m = new THREE.Matrix4();
  // Matrix4.set 은 row-major
  m.set(
    r[0][0], r[0][1], r[0][2], t[0][0],
    r[1][0], r[1][1], r[1][2], t[1][0],
    r[2][0], r[2][1], r[2][2], t[2][0],
    0, 0, 0, 1,
  );
  return m;
}

interface Scene3DLayerProps {
  robotId: string;
}

export function Scene3DLayer({ robotId }: Scene3DLayerProps) {
  const enabled = useScanStore((s) => s.liveEnabled);
  const pointSize = useScanStore((s) => s.pointSize);
  const { robots } = useRobots();

  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId });

  // hand_eye — Mirror (snapshot + CALIBRATION_ACTIVATED invalidate+refetch).
  // calibration 은 robot-agnostic — 대상 robot 은 req 필드 (§2.7).
  const bundle = useMirror({
    snapshotService: ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE,
    snapshotReq: { robot_id: robotId },
    changeTopic: Topic.CALIBRATION_ACTIVATED,
    robotId,
  });

  const handEye = useMemo(
    () => handEyeMatrix(bundle.value as CalibrationBundle | null),
    [bundle.value],
  );

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
    if (!enabled) {
      geom.setDrawRange(0, 0);
      return;
    }
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
  }, [enabled, robotId, geom]);

  // world 배치 = base(base_pose) · tcp(corrected FK) · hand_eye
  const transform = useMemo(() => {
    if (!tcp.value) return null;
    const robot = robots.find((r) => r.id === robotId);
    if (!robot) return null;
    const base = robotBaseMatrix(robot.base_pose);
    const tcpWorld = poseToWorldMatrix(base, tcp.value.position, tcp.value.quaternion);
    const m = tcpWorld.multiply(handEye);
    const p = new THREE.Vector3();
    const q = new THREE.Quaternion();
    const s = new THREE.Vector3();
    m.decompose(p, q, s);
    return { position: [p.x, p.y, p.z] as const, quaternion: [q.x, q.y, q.z, q.w] as const };
  }, [tcp.value, robots, robotId, handEye]);

  if (!enabled || !transform) return null;

  return (
    <points
      frustumCulled={false}
      position={[transform.position[0], transform.position[1], transform.position[2]]}
      quaternion={[
        transform.quaternion[0],
        transform.quaternion[1],
        transform.quaternion[2],
        transform.quaternion[3],
      ]}
    >
      <primitive object={geom} attach="geometry" />
      <pointsMaterial size={pointSize / 1000} vertexColors sizeAttenuation />
    </points>
  );
}
