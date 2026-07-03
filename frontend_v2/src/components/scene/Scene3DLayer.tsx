/**
 * Scene3DLayer — 라이브 point cloud (backend scene3d module).
 *
 * scene3d 가 camera-frame xyz+rgb 를 발행 (Scene3dCloud, msgpack bin) → 여기서
 * 부모 transform = tcpMatrix · handEye 로 world 배치 (옛 backend Scene3DLayer 패턴).
 * hand_eye 는 calibration bundle 에서 1회 fetch (없으면 identity fallback — mock).
 *
 * liveEnabled(scanStore) gate — scan 모드에서 [라이브] 켤 때만 구독.
 */
import { useEffect, useMemo } from "react";
import * as THREE from "three";
import { bridge, decodeMsgpackRecord, topicFor } from "@/api/bridge";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type { CalibrationBundle } from "@/api/generated/contract";
import { useService } from "@/framework";
import { useScanStore } from "@/stores/scanStore";

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
  tcpMatrix: THREE.Matrix4 | null;
  robotId: string;
}

export function Scene3DLayer({ tcpMatrix, robotId }: Scene3DLayerProps) {
  const enabled = useScanStore((s) => s.liveEnabled);
  const bundle = useService(ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE, robotId);

  // hand_eye 1회 fetch (라이브 켤 때 최신값 반영) — calibration 은 robot-agnostic,
  // 대상 robot 은 req 필드.
  useEffect(() => {
    if (enabled) void bundle.call({ robot_id: robotId });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [enabled, robotId]);

  const handEye = useMemo(
    () => handEyeMatrix(bundle.data as CalibrationBundle | null),
    [bundle.data],
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

  const transform = useMemo(() => {
    if (!tcpMatrix) return null;
    const m = tcpMatrix.clone().multiply(handEye);
    const p = new THREE.Vector3();
    const q = new THREE.Quaternion();
    const s = new THREE.Vector3();
    m.decompose(p, q, s);
    return { position: [p.x, p.y, p.z] as const, quaternion: [q.x, q.y, q.z, q.w] as const };
  }, [tcpMatrix, handEye]);

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
      <pointsMaterial size={0.0025} vertexColors sizeAttenuation />
    </points>
  );
}
