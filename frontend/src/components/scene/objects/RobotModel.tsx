/**
 * URDF robot model. 옛 frontend RobotModel.tsx 의 ref-stash + loadMeshCb override
 * pattern 그대로 carry over — commit f15a20b 의 race fix + cross-robot opacity bleed
 * fix 보존 (frontend.md §10 + anchor #12).
 *
 * 변경 사항 (backend 정합):
 *   - useMotorConfigs 제거 — backend 의 Motion.Stream.TCP_STATE.joints (rad list)
 *     활용. joint name 매핑은 URDF 의 non-fixed joint file order 그대로.
 *   - URDF path: `${BASE_URL}/robot/${type}/urdf/${type}.urdf` (Bridge static mount).
 */
import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import type { URDFRobot } from "urdf-loader";
import { BASE_URL } from "@/constants";
import type { BasePoseInfo } from "@/api/generated/contract";
import { applyJoints } from "./jointMapping";

interface URDFRobotProps {
  /** URDF type — `/robot/<type>/urdf/<type>.urdf` 경로 추론. */
  robotType: string;
  /** arm joint name list (backend TcpState.joint_names SSOT). jointAngles 와 same index.
   *  URDF 파일의 joint 선언 순서는 믿지 X — motors.yaml 순서가 진짜 SSOT. */
  jointNames: string[];
  /** arm joint angle list (rad). jointNames 와 parallel array. */
  jointAngles: number[];
  /** World frame robot base 위치 (m). multi-robot 동시 마운트 시 분리. */
  basePose?: BasePoseInfo;
  /** dim — 1.0 불투명, 0.25 정도가 "다른 로봇 흐릿하게" default. */
  opacity?: number;
  /** material color overlay (hex). ghost preview 등. null=원본. */
  tint?: string | null;
  onLinksLoaded?: (linkNames: string[]) => void;
  /** 로드 후 로봇 바운딩스피어(월드) 보고 — 카메라/그리드 auto-fit 용. */
  onBounds?: (radius: number, center: [number, number, number]) => void;
  linkVisibility?: Record<string, boolean>;
  visible?: boolean;
}

function paintMaterial(
  m: THREE.Material,
  opacity: number,
  tint: string | null | undefined,
) {
  const transparent = opacity < 1.0;
  // transparent 토글은 shader program 재컴파일 필요 — needsUpdate 없이는 이미
  // 렌더된 material 에 안 먹힘 (fresh load 는 첫 렌더 전에 칠해져서 무증상,
  // robot 간 네비게이션의 focus 전환에서만 드러나던 잠복 버그 — 2026-07-09).
  if (m.transparent !== transparent) m.needsUpdate = true;
  m.transparent = transparent;
  m.opacity = opacity;
  m.depthWrite = !transparent;
  const colored = m as THREE.Material & { color?: THREE.Color };
  if (tint && colored.color) colored.color.set(tint);
}

function applyMaterialProps(
  robot: URDFRobot,
  opacity: number,
  tint: string | null | undefined,
) {
  robot.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (!mesh.isMesh) return;
    const mat = mesh.material as THREE.Material | THREE.Material[];
    if (Array.isArray(mat)) mat.forEach((m) => paintMaterial(m, opacity, tint));
    else if (mat) paintMaterial(mat, opacity, tint);
  });
}

function disposeMaterials(robot: URDFRobot) {
  robot.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (!mesh.isMesh) return;
    const mat = mesh.material as THREE.Material | THREE.Material[];
    if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
    else if (mat) mat.dispose();
  });
}

export function RobotModel({
  robotType,
  jointNames,
  jointAngles,
  basePose,
  opacity = 1.0,
  tint = null,
  onLinksLoaded,
  onBounds,
  linkVisibility,
  visible = true,
}: URDFRobotProps) {
  const groupRef = useRef<THREE.Group>(null);
  const robotRef = useRef<URDFRobot>(null);
  // URDF 로드 완료를 reactive state 로 lift — robotRef 는 imperative 라 effect dep
  // 가 못 됨. 이 flag 가 false→true 되면 joint/opacity/visibility effect 들이 re-run.
  const [robotReady, setRobotReady] = useState(false);

  // opacity / tint / jointAngles 를 ref 로 stash — URDFLoader 의 loadMeshCb 는 async
  // 라 효 시점의 *현재* 값을 써야 늦게 들어오는 mesh 도 정확히 dim/tint 처리.
  const opacityRef = useRef(opacity);
  useEffect(() => {
    opacityRef.current = opacity;
  }, [opacity]);
  const tintRef = useRef(tint);
  useEffect(() => {
    tintRef.current = tint;
  }, [tint]);
  const jointAnglesRef = useRef(jointAngles);
  useEffect(() => {
    jointAnglesRef.current = jointAngles;
  }, [jointAngles]);
  const jointNamesRef = useRef(jointNames);
  useEffect(() => {
    jointNamesRef.current = jointNames;
  }, [jointNames]);

  // URDF 로드 + loadMeshCb override (cross-robot material clone — commit f15a20b race fix).
  useEffect(() => {
    let cancelled = false;
    const currentGroup = groupRef.current;

    const loader = new URDFLoader();
    loader.packages = {
      [robotType]: `${BASE_URL}/robot`,
    };
    loader.workingPath = `${BASE_URL}/robot/${robotType}/urdf/`;

    const paint = (m: THREE.Material) =>
      paintMaterial(m, opacityRef.current, tintRef.current);
    loader.loadMeshCb = (path, manager, urdfDone) => {
      loader.defaultMeshLoader(path, manager, (obj, err) => {
        urdfDone(obj, err);
        if (err || !obj) return;
        // .stl = 단일 Mesh / .dae(COLLADA) = Group(자식 Mesh들). traverse 로 둘 다
        // 커버 — mesh 도착 시점에 material 을 인스턴스별 clone + paint 해야 tint/
        // opacity 가 확실히 먹는다 (.dae 를 top-level Mesh 로만 보면 Group 이라
        // 건너뛰어, 로드 타이밍상 applyMaterialProps 도 놓쳐 고스트가 불투명 원본으로
        // 나오던 버그 — ur5e .dae). clone 은 cross-robot material bleed 도 차단.
        obj.traverse((o) => {
          const mesh = o as THREE.Mesh;
          if (!mesh.isMesh) return;
          const mat = mesh.material as THREE.Material | THREE.Material[];
          if (Array.isArray(mat)) {
            mesh.material = mat.map((m) => {
              const c = m.clone();
              paint(c);
              return c;
            });
          } else if (mat) {
            const c = (mat as THREE.Material).clone();
            paint(c);
            mesh.material = c;
          }
        });
      });
    };

    loader.load(
      `${BASE_URL}/robot/${robotType}/urdf/${robotType}.urdf`,
      (robot: URDFRobot) => {
        if (cancelled) return;
        robotRef.current = robot;
        // scene 에 add 하기 *전에* 최신 joint 적용 — default pose flash 차단.
        applyJoints(robot, jointNamesRef.current, jointAnglesRef.current);
        currentGroup?.add(robot);
        if (robot.links) {
          const names = Object.keys(robot.links).sort();
          onLinksLoaded?.(names);
        }
        setRobotReady(true);
      },
      undefined,
      (err: unknown) => console.error("[RobotModel] URDF load error:", err),
    );

    return () => {
      cancelled = true;
      if (robotRef.current && currentGroup) {
        disposeMaterials(robotRef.current);
        currentGroup.remove(robotRef.current);
        robotRef.current = null;
      }
      setRobotReady(false);
    };
  }, [robotType, onLinksLoaded]);

  // Joint 각도 적용 — robotReady dep 로 URDF 늦게 로드된 자리도 mount 직후 최신 동기화.
  useEffect(() => {
    const robot = robotRef.current;
    if (!robot) return;
    applyJoints(robot, jointNames, jointAngles);
  }, [jointNames, jointAngles, robotReady]);

  useEffect(() => {
    if (robotRef.current) robotRef.current.visible = visible;
  }, [visible, robotReady]);

  useEffect(() => {
    if (robotRef.current) applyMaterialProps(robotRef.current, opacity, tint);
  }, [opacity, tint, robotReady]);

  useEffect(() => {
    const robot = robotRef.current;
    if (!robot?.links || !linkVisibility) return;
    Object.entries(linkVisibility).forEach(([name, vis]) => {
      const link = robot.links[name];
      if (link) link.visible = vis;
    });
  }, [linkVisibility, robotReady]);

  // 로봇 실제 크기(월드 바운딩스피어) 측정 → 카메라/그리드 auto-fit.
  // 메시(.dae/.stl)가 하나씩 async 로 들어오므로, 프레임마다 재서 바운딩이 더 이상
  // 안 커질 때(=모든 메시 로드 완료)까지 기다렸다 1회 보고. 첫 non-empty 에 쏘면
  // 베이스 링크만 잡혀 크기가 작게 나옴(카메라 코앞 버그). onBounds 는 안정된 값만.
  useEffect(() => {
    if (!robotReady || !onBounds) return;
    let raf = 0;
    let tries = 0;
    let lastR = -1;
    let stable = 0;
    const report = (radius: number, c: THREE.Vector3) => {
      onBounds(radius, [c.x, c.y, c.z]);
    };
    const tick = () => {
      const robot = robotRef.current;
      if (robot) {
        robot.updateWorldMatrix(true, true);
        const box = new THREE.Box3().setFromObject(robot);
        if (!box.isEmpty()) {
          const s = box.getBoundingSphere(new THREE.Sphere());
          if (Number.isFinite(s.radius) && s.radius > 0) {
            if (Math.abs(s.radius - lastR) < 1e-3) {
              // 8프레임 연속 안 커지면 메시 로드 끝 → 확정 보고
              if (++stable >= 8) return report(s.radius, s.center);
            } else {
              stable = 0;
              lastR = s.radius;
            }
          }
        }
      }
      if (tries++ < 300) raf = requestAnimationFrame(tick);
      else if (lastR > 0 && robotRef.current) {
        // 안전망: 5초 내 안정 안 돼도 마지막 측정으로 fit
        const box = new THREE.Box3().setFromObject(robotRef.current);
        const s = box.getBoundingSphere(new THREE.Sphere());
        if (s.radius > 0) report(s.radius, s.center);
      }
    };
    tick();
    return () => cancelAnimationFrame(raf);
  }, [robotReady, onBounds]);

  // World transform: base_pose 적용 (z-up world) + URDF z-up→y-up 보정.
  // outer group: [-π/2 rot around X] — z-up → y-up.
  // inner group: base_pose translation + yaw (z-up world 회전).
  const px = basePose?.x ?? 0;
  const py = basePose?.y ?? 0;
  const pz = basePose?.z ?? 0;
  const yawRad = ((basePose?.yaw_deg ?? 0) * Math.PI) / 180;

  return (
    <group rotation={[-Math.PI / 2, 0, 0]}>
      <group position={[px, py, pz]} rotation={[0, 0, yawRad]}>
        <group ref={groupRef} />
      </group>
    </group>
  );
}
