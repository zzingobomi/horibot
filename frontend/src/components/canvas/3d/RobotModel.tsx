import { useEffect, useRef } from "react";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import type { URDFRobot } from "urdf-loader";
import { BASE_URL } from "@/constants";
import { TCP_LINK_NAME, JOINT_CONFIGS } from "@/lib/robot/config";

export interface RobotBasePose {
  x: number;
  y: number;
  z: number;
  yaw_deg: number;
}

interface URDFRobotProps {
  jointAngles: number[];
  /** URDF 의 robot_type — `robot/<type>/urdf/<type>.urdf` 경로 추론. 기본: "omx_f". */
  robotType?: string;
  /** World frame 기준 robot base 위치 (m). 두 URDF 동시 마운트 시 겹치지 않게 분리. */
  basePose?: RobotBasePose;
  /** dim 효과 — 1.0 불투명, 0.3 정도가 "다른 로봇 흐릿하게" 의 합리적 default. */
  opacity?: number;
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

function applyOpacity(robot: URDFRobot, opacity: number) {
  robot.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (!mesh.isMesh) return;
    const mat = mesh.material as THREE.Material | THREE.Material[];
    const apply = (m: THREE.Material) => {
      m.transparent = opacity < 1.0;
      m.opacity = opacity;
      m.depthWrite = opacity >= 1.0;
    };
    if (Array.isArray(mat)) mat.forEach(apply);
    else if (mat) apply(mat);
  });
}

export function RobotModel({
  jointAngles,
  robotType = "omx_f",
  basePose,
  opacity = 1.0,
  onTCPMatrix,
  onLinksLoaded,
  linkVisibility,
  visible = true,
}: URDFRobotProps) {
  const groupRef = useRef<THREE.Group>(null);
  const robotRef = useRef<URDFRobot>(null);
  const onTCPMatrixRef = useRef(onTCPMatrix);
  useEffect(() => {
    onTCPMatrixRef.current = onTCPMatrix;
  }, [onTCPMatrix]);

  // URDF 로드 — robot_type 별로 분기, packages 매핑은 동일 (현재 모든 type 이
  // robot/<type>/ 하위에 mesh 보유).
  useEffect(() => {
    let cancelled = false;
    const currentGroup = groupRef.current;

    const loader = new URDFLoader();
    loader.packages = {
      omx_description: `${BASE_URL}/robot`,
      [robotType]: `${BASE_URL}/robot`,
    };
    loader.workingPath = `${BASE_URL}/robot/${robotType}/urdf/`;

    loader.load(
      `${BASE_URL}/robot/${robotType}/urdf/${robotType}.urdf`,
      (robot: URDFRobot) => {
        if (cancelled) return;

        robotRef.current = robot;
        currentGroup?.add(robot);

        if (robot.links) {
          const names = Object.keys(robot.links).sort();
          onLinksLoaded?.(names);
        }

        applyOpacity(robot, opacity);
        emitTCP(robot, onTCPMatrixRef.current);
      },
      undefined,
      (err: unknown) => console.error("[URDFRobot] load error:", err),
    );

    return () => {
      cancelled = true;
      if (robotRef.current && currentGroup) {
        currentGroup.remove(robotRef.current);
        robotRef.current = null;
      }
    };
    // opacity 변경 시 reload 안 함 — 별도 effect 에서 traverse.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [robotType, onLinksLoaded]);

  // Joint 각도 적용
  //
  // 무한루프 주의 — 부모 (RobotLayer / RobotScene) 가 onTCPMatrix 를 인라인
  // arrow 로 전달하면 매 렌더마다 새 함수 참조. 이 effect 의 dep 에 prop 을
  // 직접 두면: emit → 부모 setState → 리렌더 → 새 onTCPMatrix → dep 변경 →
  // effect 재실행 → emit → ... 무한 루프 → React reconciler stall → 라우팅까지
  // 막힘. emitTCP 는 latest callback ref 로만 호출하고 dep 은 jointAngles 만.
  useEffect(() => {
    const robot = robotRef.current;
    if (!robot) return;

    JOINT_CONFIGS.forEach((joint, i) => {
      const angle = jointAngles[i];
      if (angle !== undefined && robot.joints?.[joint.name]) {
        robot.setJointValue(joint.name, angle);
      }
    });

    emitTCP(robot, onTCPMatrixRef.current);
  }, [jointAngles]);

  // 전체 visible
  useEffect(() => {
    if (robotRef.current) robotRef.current.visible = visible;
  }, [visible]);

  // Opacity (focus 모드 dim others)
  useEffect(() => {
    if (robotRef.current) applyOpacity(robotRef.current, opacity);
  }, [opacity]);

  // 링크별 visibility
  useEffect(() => {
    const robot = robotRef.current;
    if (!robot?.links || !linkVisibility) return;

    Object.entries(linkVisibility).forEach(([name, vis]) => {
      const link = robot.links[name];
      if (link) link.visible = vis;
    });
  }, [linkVisibility]);

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
