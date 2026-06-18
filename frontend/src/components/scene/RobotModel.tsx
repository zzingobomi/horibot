import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import type { URDFRobot } from "urdf-loader";
import { BASE_URL } from "@/constants";
import {
  TCP_LINK_NAME,
  useMotorConfigs,
  type MotorConfigItem,
} from "@/lib/robot/config";
import type { RobotBasePose } from "@/types/robot";

interface URDFRobotProps {
  jointAngles: number[];
  /** URDF 의 robot_type — `robot/<type>/urdf/<type>.urdf` 경로 추론. 기본: "omx_f". */
  robotType?: string;
  /** robot instance id — motor config (joint name 매핑) lookup 에 사용. */
  robotId?: string;
  /** World frame 기준 robot base 위치 (m). 두 URDF 동시 마운트 시 겹치지 않게 분리. */
  basePose?: RobotBasePose;
  /** dim 효과 — 1.0 불투명, 0.3 정도가 "다른 로봇 흐릿하게" 의 합리적 default. */
  opacity?: number;
  /** material color overlay (hex). ghost preview (주황 #ff8c1a) 등. null=원본 색. */
  tint?: string | null;
  onTCPMatrix?: (matrix: THREE.Matrix4) => void;
  onLinksLoaded?: (linkNames: string[]) => void;
  linkVisibility?: Record<string, boolean>;
  visible?: boolean;
}

function emitTCP(
  robot: URDFRobot,
  cb: ((m: THREE.Matrix4) => void) | undefined,
) {
  if (!cb || !robot.links?.[TCP_LINK_NAME]) return;
  const link = robot.links[TCP_LINK_NAME];
  link.updateWorldMatrix(true, false);
  cb(link.matrixWorld.clone());
}

/** material 1개에 opacity (+ 선택적 tint color overlay) 적용. */
function paintMaterial(
  m: THREE.Material,
  opacity: number,
  tint: string | null | undefined,
) {
  m.transparent = opacity < 1.0;
  m.opacity = opacity;
  m.depthWrite = opacity >= 1.0;
  // .color 있는 material (Standard/Phong/Basic) 만 tint. clone 된 인스턴스라
  // robot 별 색 안전 (loadMeshCb 가 mesh 마다 clone).
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

function applyJoints(
  robot: URDFRobot,
  cfgs: MotorConfigItem[],
  angles: number[],
) {
  cfgs.forEach((cfg, i) => {
    const angle = angles[i];
    if (angle !== undefined && robot.joints?.[cfg.name]) {
      robot.setJointValue(cfg.name, angle);
    }
  });
}

export function RobotModel({
  jointAngles,
  robotType = "omx_f",
  robotId,
  basePose,
  opacity = 1.0,
  tint = null,
  onTCPMatrix,
  onLinksLoaded,
  linkVisibility,
  visible = true,
}: URDFRobotProps) {
  const motorCfgs = useMotorConfigs(robotId);
  const groupRef = useRef<THREE.Group>(null);
  const robotRef = useRef<URDFRobot>(null);
  // URDF 로드 완료를 reactive state 로 lift — robotRef 는 imperative 라 effect
  // dep 가 못 됨. 이 flag 가 false→true 되면 joint/opacity/visibility effect
  // 들이 re-run 하면서 URDF mount 직후 최신 상태로 동기화. 안 두면 URDF 가
  // 늦게 로드된 자리는 다음 topic update 까지 default pose 로 잠시 보인다.
  const [robotReady, setRobotReady] = useState(false);
  const onTCPMatrixRef = useRef(onTCPMatrix);
  useEffect(() => {
    onTCPMatrixRef.current = onTCPMatrix;
  }, [onTCPMatrix]);

  // opacity 를 ref 로 stash — URDFLoader 의 loadMeshCb 는 mesh 가 async 로 들어올
  // 때마다 호출되는데 그 시점의 *현재* opacity 를 써야 늦게 들어오는 mesh 도
  // dim 처리가 정확함.
  const opacityRef = useRef(opacity);
  useEffect(() => {
    opacityRef.current = opacity;
  }, [opacity]);

  // tint 도 ref stash — loadMeshCb (async) 가 늦게 들어오는 mesh 에도 현재 tint 적용.
  const tintRef = useRef(tint);
  useEffect(() => {
    tintRef.current = tint;
  }, [tint]);

  // URDF load callback (async) 안에서 *현재* joint state 를 적용하려면 ref 로
  // stash 해야 함 — effect closure 의 값은 마운트 시점 snapshot.
  // scene 에 add 하기 전에 joint 적용 = 한 프레임 default pose flash 차단.
  const motorCfgsRef = useRef(motorCfgs);
  const jointAnglesRef = useRef(jointAngles);
  useEffect(() => {
    motorCfgsRef.current = motorCfgs;
  }, [motorCfgs]);
  useEffect(() => {
    jointAnglesRef.current = jointAngles;
  }, [jointAngles]);

  // URDF 로드.
  //
  // loadMeshCb override 이유: URDFLoader 가 URDF `<material name="X">` 을 robot
  // 안 모든 mesh 에 같은 인스턴스로 공유시키고, 그것도 mesh 가 async 로 attach
  // 되며 URDF.load 의 onComplete 는 그보다 먼저 fire. mesh 별로 자기 material
  // 인스턴스 + 현재 opacity 가져가야 (a) 두 robot 의 opacity 가 서로 안 덮어쓰고
  // (b) 늦게 들어오는 mesh 도 attach 직후부터 정확한 dim. loadMeshCb 안에서
  // URDFLoader 의 done 호출 *후* (그 시점에 obj.material 이 채워짐) clone + set.
  // opacity 는 ref 로 — useEffect closure 의 stale value 회피.
  useEffect(() => {
    let cancelled = false;
    const currentGroup = groupRef.current;

    const loader = new URDFLoader();
    loader.packages = {
      omx_description: `${BASE_URL}/robot`,
      [robotType]: `${BASE_URL}/robot`,
    };
    loader.workingPath = `${BASE_URL}/robot/${robotType}/urdf/`;

    const paint = (m: THREE.Material) =>
      paintMaterial(m, opacityRef.current, tintRef.current);
    loader.loadMeshCb = (path, manager, urdfDone) => {
      loader.defaultMeshLoader(path, manager, (obj, err) => {
        urdfDone(obj, err);
        if (err || !obj) return;
        const mesh = obj as THREE.Mesh;
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
    };

    loader.load(
      `${BASE_URL}/robot/${robotType}/urdf/${robotType}.urdf`,
      (robot: URDFRobot) => {
        if (cancelled) return;
        robotRef.current = robot;
        // scene 에 add 하기 *전에* 최신 joint 적용 — 안 그러면 한 프레임 동안
        // URDF default pose (모든 joint=0) 가 보임. Three.js 가 add 직후 다음
        // 프레임에 렌더 → 그 시점에 이미 setJointValue 끝나 있어야 깨끗하다.
        applyJoints(robot, motorCfgsRef.current, jointAnglesRef.current);
        currentGroup?.add(robot);
        if (robot.links) {
          const names = Object.keys(robot.links).sort();
          onLinksLoaded?.(names);
        }
        emitTCP(robot, onTCPMatrixRef.current);
        setRobotReady(true);
      },
      undefined,
      (err: unknown) => console.error("[URDFRobot] load error:", err),
    );

    return () => {
      cancelled = true;
      if (robotRef.current && currentGroup) {
        // cloned material dispose — robot 별로 clone 했으니 unmount 시 정리.
        disposeMaterials(robotRef.current);
        currentGroup.remove(robotRef.current);
        robotRef.current = null;
      }
      setRobotReady(false);
    };
  }, [robotType, onLinksLoaded]);

  // Joint 각도 적용
  //
  // 무한루프 주의 — 부모 (RobotLayer / RobotScene) 가 onTCPMatrix 를 인라인
  // arrow 로 전달하면 매 렌더마다 새 함수 참조. 이 effect 의 dep 에 prop 을
  // 직접 두면: emit → 부모 setState → 리렌더 → 새 onTCPMatrix → dep 변경 →
  // effect 재실행 → emit → ... 무한 루프 → React reconciler stall → 라우팅까지
  // 막힘. emitTCP 는 latest callback ref 로만 호출하고 dep 은 jointAngles 만.
  //
  // robotReady 도 dep — URDF 가 늦게 로드된 자리에서 mount 직후 최신 joint
  // 적용 (안 두면 다음 topic update 50ms 까지 URDF default pose 잠깐 보임).
  useEffect(() => {
    const robot = robotRef.current;
    if (!robot) return;
    applyJoints(robot, motorCfgs, jointAngles);
    emitTCP(robot, onTCPMatrixRef.current);
  }, [jointAngles, motorCfgs, robotReady]);

  // 전체 visible
  useEffect(() => {
    if (robotRef.current) robotRef.current.visible = visible;
  }, [visible, robotReady]);

  // Opacity (focus 모드 dim others) + tint (ghost preview)
  useEffect(() => {
    if (robotRef.current) applyMaterialProps(robotRef.current, opacity, tint);
  }, [opacity, tint, robotReady]);

  // 링크별 visibility
  useEffect(() => {
    const robot = robotRef.current;
    if (!robot?.links || !linkVisibility) return;

    Object.entries(linkVisibility).forEach(([name, vis]) => {
      const link = robot.links[name];
      if (link) link.visible = vis;
    });
  }, [linkVisibility, robotReady]);

  // World transform: base_pose 적용 (z-up world) + URDF 자체는 z-up→y-up 보정.
  // 부모 group 이 base_pose, 자식 group 이 URDF 회전.
  const px = basePose?.x ?? 0;
  const py = basePose?.y ?? 0;
  const pz = basePose?.z ?? 0;
  const yawRad = ((basePose?.yaw_deg ?? 0) * Math.PI) / 180;

  // R3F 의 y-up world 에 z-up world 를 끼우는 형태:
  // outer group: [-π/2 rot around X] 로 z-up→y-up 변환.
  // inner group: base_pose translation + yaw (z-up world 의 회전).
  // → 두 robot 의 base 위치가 z-up world 의 (x, y, z) 로 자연스럽게 표현.
  return (
    <group rotation={[-Math.PI / 2, 0, 0]}>
      <group position={[px, py, pz]} rotation={[0, 0, yawRad]}>
        <group ref={groupRef} />
      </group>
    </group>
  );
}
