/**
 * ScanGrowth — world_scan 진행 중 "월드가 자라는" 성장 UX (씬 객체).
 *
 * 설계 (2026-07-21, 사용자 아이디어): world_scan 은 빌드를 **끝에 1번**만 한다
 * (전체 재빌드라 pose 마다 빌드 = 낭비). 성장 애니메이션은 mesh 빌드가 아니라
 * **포즈별 라이브 포인트클라우드 스냅샷을 base frame 에 누적**해서 보여준다
 * (점 표시는 공짜 — ICP/TSDF 없음). 스캔이 끝나면(STATE≠running) 누적을 비우고
 * World(최종 mesh)가 이어받는다.
 *
 * 좌표 (correct-by-construction): Camera live cloud 와 **같은 변환 체인**을 쓴다 —
 * cloud 점은 camera frame, cameraInBase(tcp·handEye) 로 base(RobotFrame-local) 로
 * 옮겨 저장, <RobotFrame> 부모가 base_pose 로 월드 배치. 스냅샷 시점의 tcp·캘을
 * 점에 구워넣으므로(freeze) 로봇이 다음 포즈로 움직여도 이전 점은 그 자리에 남는다.
 *
 * 스냅샷 시점 (mid-motion smear 회피): 로봇이 **정지**(직전 프레임 대비 이동
 * 미미)했고 **직전 keyframe 에서 충분히 이동**했을 때만 1 keyframe 누적 →
 * 포즈당 대략 1번, 이동 중 프레임은 skip.
 *
 * ⚠️ 홈-검증 필요: 변환 정확도·정지 판정 임계·smear 는 실물 스캔에서만 확정된다
 * (sim/vitest 로 3D 시각 검증 불가). 이 파일은 그 검증 대상 — 로그로 keyframe
 * 누적을 남긴다 (안 자라거나 어긋나면 그 데이터로 진단).
 */
import { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { bridge, decodeMsgpackRecord, topicFor } from "@/api/bridge";
import { useMirror, useStream } from "@/framework";
import {
  ServiceKey,
  TaskStatus,
  Topic,
  type CalibrationBundle,
} from "@/api/generated/contract";
import { RobotFrame } from "../shared/RobotFrame";
import { cameraInBase } from "./cameraPose";
import type { SceneObjectProps } from "../sceneTypes";

const MAX_ACCUM = 1_500_000; // 누적 점 상한 (BufferGeometry 1회 할당)
const STILL_M = 0.004; // 직전 프레임 대비 이 이하 이동 = 정지 (keyframe 자격)
const KEYFRAME_MOVE_M = 0.03; // 직전 keyframe 에서 이 이상 이동해야 새 keyframe
const POINT_SIZE_M = 0.002;

export function ScanGrowth({ robots, focusId }: SceneObjectProps) {
  const robotId = focusId ?? robots[0]?.id ?? "";
  const ws = useStream(Topic.WORLDSCAN_STATE, { robotId });
  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId });
  const bundleM = useMirror({
    snapshotService: ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE,
    snapshotReq: { robot_id: robotId },
    changeTopic: Topic.CALIBRATION_ACTIVATED,
    robotId,
  });
  const bundle = bundleM.value as CalibrationBundle | null;

  const running = ws.value?.status === TaskStatus.RUNNING;

  // 최신 tcp/bundle 을 콜백에서 읽기 위한 ref (cloud 콜백 클로저가 stale 안 되게).
  const tcpRef = useRef(tcp.value);
  const bundleRef = useRef(bundle);
  useEffect(() => {
    tcpRef.current = tcp.value;
    bundleRef.current = bundle;
  }, [tcp.value, bundle]);

  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    const pos = new THREE.BufferAttribute(new Float32Array(MAX_ACCUM * 3), 3);
    const col = new THREE.BufferAttribute(new Float32Array(MAX_ACCUM * 3), 3);
    pos.setUsage(THREE.DynamicDrawUsage);
    col.setUsage(THREE.DynamicDrawUsage);
    g.setAttribute("position", pos);
    g.setAttribute("color", col);
    g.setDrawRange(0, 0);
    return g;
  }, []);

  const writeRef = useRef(0); // 누적 점 수 (draw range 끝)
  const lastKfRef = useRef<THREE.Vector3 | null>(null); // 직전 keyframe tcp 위치
  const prevPosRef = useRef<THREE.Vector3 | null>(null); // 직전 프레임 tcp 위치

  // 스캔이 멈추면(또는 시작) 누적 리셋 — World(mesh)가 이어받는다.
  useEffect(() => {
    if (running) return; // 시작 시엔 유지된 채로 두고, 끝/idle 에서 clear
    writeRef.current = 0;
    lastKfRef.current = null;
    prevPosRef.current = null;
    geom.setDrawRange(0, 0);
    geom.getAttribute("position").needsUpdate = true;
  }, [running, geom]);

  useEffect(() => {
    if (!running || !robotId) return;
    const wire = topicFor(Topic.SCENE3D_CLOUD, robotId);
    const unsub = bridge.subscribeBinary(wire, (buf) => {
      const t = tcpRef.current;
      if (!t) return;
      const cur = new THREE.Vector3(t.position[0], t.position[1], t.position[2]);
      const prev = prevPosRef.current;
      prevPosRef.current = cur.clone();
      // 정지 판정 — 직전 프레임에서 거의 안 움직였나 (이동 중 프레임 skip).
      if (!prev || cur.distanceTo(prev) > STILL_M) return;
      // 새 포즈 판정 — 직전 keyframe 에서 충분히 이동했나.
      if (lastKfRef.current && cur.distanceTo(lastKfRef.current) < KEYFRAME_MOVE_M) {
        return;
      }
      const rec = decodeMsgpackRecord(buf) as unknown as {
        point_count: number;
        xyz_bytes: Uint8Array;
        rgb_bytes: Uint8Array;
      };
      const n = rec.point_count ?? 0;
      if (!n) return;
      // camera→base(local) 변환 — live cloud 와 같은 체인 (스냅샷 시점 tcp·캘 freeze).
      const cam = cameraInBase(t.position, t.quaternion, bundleRef.current);
      const m = new THREE.Matrix4().compose(
        new THREE.Vector3(cam.position[0], cam.position[1], cam.position[2]),
        new THREE.Quaternion(
          cam.quaternion[0], cam.quaternion[1], cam.quaternion[2], cam.quaternion[3],
        ),
        new THREE.Vector3(1, 1, 1),
      );
      const xyzU8 = new Uint8Array(rec.xyz_bytes);
      const xyz = new Float32Array(xyzU8.buffer, 0, n * 3);
      const rgb = rec.rgb_bytes;
      const posArr = geom.getAttribute("position").array as Float32Array;
      const colArr = geom.getAttribute("color").array as Float32Array;
      const v = new THREE.Vector3();
      let w = writeRef.current;
      let added = 0;
      for (let i = 0; i < n && w < MAX_ACCUM; i++) {
        v.set(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]).applyMatrix4(m);
        posArr[w * 3] = v.x;
        posArr[w * 3 + 1] = v.y;
        posArr[w * 3 + 2] = v.z;
        colArr[w * 3] = rgb[i * 3] / 255;
        colArr[w * 3 + 1] = rgb[i * 3 + 1] / 255;
        colArr[w * 3 + 2] = rgb[i * 3 + 2] / 255;
        w++;
        added++;
      }
      writeRef.current = w;
      lastKfRef.current = cur.clone();
      geom.getAttribute("position").needsUpdate = true;
      geom.getAttribute("color").needsUpdate = true;
      geom.setDrawRange(0, w);
      geom.computeBoundingSphere();
      // 관측성 — keyframe 누적 로그 (안 자라거나 어긋나면 집에서 이 데이터로 진단).
      console.debug(
        `[ScanGrowth] keyframe: +${added}pts → ${w} at ` +
          `(${cur.x.toFixed(2)},${cur.y.toFixed(2)},${cur.z.toFixed(2)})` +
          (w >= MAX_ACCUM ? " [상한 도달 — 이후 skip]" : ""),
      );
    });
    return unsub;
  }, [running, robotId, geom]);

  // 누적 point 는 geom.drawRange 로 관리 (running=false 면 위 effect 가 0 으로
  // 리셋 → World mesh 가 담당). points 노드는 항상 렌더, draw range 가 gate.
  return (
    <RobotFrame robotId={robotId}>
      <points frustumCulled={false}>
        <primitive object={geom} attach="geometry" />
        <pointsMaterial size={POINT_SIZE_M} vertexColors sizeAttenuation />
      </points>
    </RobotFrame>
  );
}
