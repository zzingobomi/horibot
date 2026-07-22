/**
 * WorkcellRoiScenePart — ROI 박스 3D 표현 + 직접 편집 (draft 렌더).
 *
 * 렌더 3계층 (Unity BoxBoundsHandle / CAD 관례 — pnp_scenario_rework §9.1-3):
 *   - 와이어  : EdgesGeometry 12모서리 (material wireframe 은 삼각 대각선이 생겨
 *               지저분 — 박스 모서리만). dirty(미저장) = TARGET 색으로 경고.
 *   - 반투명 면: 6개 **독립** plane — 한 면 = 한 bound = 한 핸들 = 한 하이라이트
 *               (1:1 매핑이 면별 강조를 공짜로). depthWrite off — 투명 볼륨이
 *               depth 를 쓰면 뒤 포인트클라우드가 가려진다. DoubleSide — 카메라가
 *               박스 안에 들어와도 면이 보인다.
 *   - 핸들    : 면 자체가 핸들 (drag = 그 bound resize, **Shift+drag = 그 축
 *               translate**) + 중앙 3축 화살표 기즈모 (drag = 축 translate,
 *               발견 가능한 주 경로 — 모디파이어 몰라도 보임).
 *
 * 드래그 = 포인터 ray ↔ 축 직선 최근접점 (dragMath.axisParamAtRay). 시작 시점의
 * roi/파라미터를 잡아두고 절대 재계산 (증분 누적이 clamp 와 얽혀 밀리는 것 방지).
 * 드래그 중 OrbitControls 비활성 (makeDefault 등록된 controls.enabled 토글).
 *
 * scenePart 규약: useRobotId + 패널 닫으면 함께 사라짐. 데이터 = workcellRoiStore
 * draft (숫자 입력과 동일 사본 — 3D 조작이 곧 필드에 반영).
 */
import { useMemo, useRef, useState, useEffect, useCallback } from "react";
import * as THREE from "three";
import { useThree, type ThreeEvent } from "@react-three/fiber";
import { useRobotId } from "@/hooks/useRobotId";
import { RobotFrame } from "@/components/scene/shared/RobotFrame";
import { VizColor } from "@/components/scene/theme/visualizationColors";
import { useWorkcellRoiStore, isRoiDirty } from "@/stores/workcellRoiStore";
import type { WorkcellRoi } from "@/api/generated/contract";
import {
  AXIS_DIR,
  FACE_AXIS,
  applyFaceDrag,
  applyTranslate,
  axisParamAtRay,
  faceCenter,
  roiCenter,
  roiSize,
  type Axis,
  type FaceId,
  type Vec3,
} from "./dragMath";

const FACES: FaceId[] = ["x_min", "x_max", "y_min", "y_max", "z_min", "z_max"];
// 면 opacity 램프 — idle 은 포인트클라우드를 거의 안 가리게, 조작 대상만 진하게.
const FACE_OPACITY_IDLE = 0.08;
const FACE_OPACITY_HOVER = 0.28;
const FACE_OPACITY_DRAG = 0.4;
// 축 화살표 색 = 좌표축 관례 (X=red/Y=green/Z=blue — VizColor 의미색과 별개 체계)
const AXIS_COLOR: Record<Axis, string> = { x: "#e5484d", y: "#46a758", z: "#3e63dd" };
const ARROW_SHAFT_M = 0.07;
const ARROW_RADIUS_M = 0.0035;

interface DragState {
  kind: "face" | "axis";
  face?: FaceId; // kind=face
  axis: Axis;
  translate: boolean; // face+Shift 또는 axis 화살표
  roi0: WorkcellRoi; // 드래그 시작 시점 draft
  anchor: Vec3; // 축 직선 기준점 (local)
  s0: number; // 시작 축 파라미터
}

/** 월드 ray → 이 scenePart 그룹의 local ray (RobotFrame 변환 흡수). */
function localRay(
  group: THREE.Group,
  e: ThreeEvent<PointerEvent>,
): { origin: Vec3; dir: Vec3 } {
  const inv = group.matrixWorld.clone().invert();
  const o = e.ray.origin.clone().applyMatrix4(inv);
  const d = e.ray.direction.clone().transformDirection(inv).normalize();
  return { origin: [o.x, o.y, o.z], dir: [d.x, d.y, d.z] };
}

/** OrbitControls enable 토글 — 드래그 중 카메라 회전 차단. 모듈 레벨 헬퍼:
 * 컴포넌트 안에서 hook 산출값을 직접 변이하면 react-hooks/immutability 위반
 * (three controls 는 외부 imperative 객체라 여기서 다루는 게 맞는 층). */
function setControlsEnabled(controls: unknown, enabled: boolean): void {
  const c = controls as { enabled?: boolean } | null;
  if (c && typeof c.enabled === "boolean") c.enabled = enabled;
}

export function WorkcellRoiScenePart() {
  const robotId = useRobotId();
  const draft = useWorkcellRoiStore((s) => s.drafts[robotId]);
  const saved = useWorkcellRoiStore((s) => s.saved[robotId]);
  if (!draft) return null; // ROI 미설정 — 패널의 "ROI 만들기"가 진입점
  return (
    <RobotFrame>
      <RoiBox robotId={robotId} roi={draft} dirty={isRoiDirty(draft, saved)} />
    </RobotFrame>
  );
}

function RoiBox({
  robotId,
  roi,
  dirty,
}: {
  robotId: string;
  roi: WorkcellRoi;
  dirty: boolean;
}) {
  const groupRef = useRef<THREE.Group>(null);
  // 수학 상태(ref — render 에서 안 읽음) / 강조 상태(state — render 가 읽음) 분리
  const dragRef = useRef<DragState | null>(null);
  const [dragTarget, setDragTarget] = useState<FaceId | Axis | null>(null);
  const [hovered, setHovered] = useState<FaceId | Axis | null>(null);
  const controls = useThree((s) => s.controls);
  const setDraft = useWorkcellRoiStore((s) => s.setDraft);
  const setActiveFace = useWorkcellRoiStore((s) => s.setActiveFace);

  const center = roiCenter(roi);
  const size = roiSize(roi);

  // 모서리 지오메트리 — bound 변경마다 재생성, 이전 것은 dispose (GPU leak 방지)
  const edges = useMemo(
    () => new THREE.EdgesGeometry(new THREE.BoxGeometry(size[0], size[1], size[2])),
    [size[0], size[1], size[2]], // eslint-disable-line react-hooks/exhaustive-deps
  );
  useEffect(() => () => edges.dispose(), [edges]);

  // 커서 — 조작 가능함을 손끝으로 (hover 시 grab)
  useEffect(() => {
    if (hovered) {
      document.body.style.cursor = "grab";
      return () => {
        document.body.style.cursor = "auto";
      };
    }
  }, [hovered]);

  const beginDrag = useCallback(
    (e: ThreeEvent<PointerEvent>, next: Omit<DragState, "s0">) => {
      const group = groupRef.current;
      if (!group) return;
      const { origin, dir } = localRay(group, e);
      const s0 = axisParamAtRay(next.anchor, AXIS_DIR[next.axis], origin, dir);
      if (s0 === null) return; // 축 ∥ 시선 — 이 각도에선 드래그 불가
      e.stopPropagation();
      (e.target as Element).setPointerCapture(e.pointerId);
      dragRef.current = { ...next, s0 };
      setDragTarget(next.face ?? next.axis);
      setControlsEnabled(controls, false);
      if (next.face) setActiveFace(robotId, next.face);
    },
    [controls, robotId, setActiveFace],
  );

  const moveDrag = useCallback(
    (e: ThreeEvent<PointerEvent>) => {
      const drag = dragRef.current;
      const group = groupRef.current;
      if (!drag || !group) return;
      e.stopPropagation();
      const { origin, dir } = localRay(group, e);
      const s = axisParamAtRay(drag.anchor, AXIS_DIR[drag.axis], origin, dir);
      if (s === null) return;
      const delta = s - drag.s0;
      // 시작 roi 기준 절대 적용 — 증분 누적의 clamp 밀림 방지
      const next = drag.translate
        ? applyTranslate(drag.roi0, drag.axis, delta)
        : applyFaceDrag(drag.roi0, drag.face as FaceId, delta);
      setDraft(robotId, next);
    },
    [robotId, setDraft],
  );

  const endDrag = useCallback(
    (e: ThreeEvent<PointerEvent>) => {
      if (!dragRef.current) return;
      e.stopPropagation();
      (e.target as Element).releasePointerCapture(e.pointerId);
      dragRef.current = null;
      setDragTarget(null);
      setControlsEnabled(controls, true);
      setActiveFace(robotId, undefined);
    },
    [controls, robotId, setActiveFace],
  );

  return (
    <group ref={groupRef}>
      {/* 와이어 — 항상 보임 (depthTest off: 점군 뒤에서도 경계 유지). dirty=미저장 경고색 */}
      <lineSegments
        geometry={edges}
        position={center}
        renderOrder={998}
      >
        <lineBasicMaterial
          color={dirty ? VizColor.TARGET : VizColor.SENSOR}
          transparent
          opacity={0.9}
          depthTest={false}
        />
      </lineSegments>

      {/* 반투명 면 6개 = resize 핸들 (Shift+drag = 그 축 translate) */}
      {FACES.map((face) => (
        <FacePlane
          key={face}
          face={face}
          roi={roi}
          highlighted={hovered === face || dragTarget === face}
          dragging={dragTarget === face}
          onOver={() => setHovered(face)}
          onOut={() => setHovered((h) => (h === face ? null : h))}
          onDown={(e) =>
            beginDrag(e, {
              kind: "face",
              face,
              axis: FACE_AXIS[face],
              translate: e.shiftKey,
              roi0: roi,
              anchor: faceCenter(roi, face),
            })
          }
          onMove={moveDrag}
          onUp={endDrag}
        />
      ))}

      {/* 중앙 3축 화살표 — 박스 전체 이동 (발견 가능한 translate 주 경로) */}
      {(["x", "y", "z"] as Axis[]).map((axis) => (
        <AxisArrow
          key={axis}
          axis={axis}
          center={center}
          highlighted={hovered === axis || dragTarget === axis}
          onOver={() => setHovered(axis)}
          onOut={() => setHovered((h) => (h === axis ? null : h))}
          onDown={(e) =>
            beginDrag(e, {
              kind: "axis",
              axis,
              translate: true,
              roi0: roi,
              anchor: center,
            })
          }
          onMove={moveDrag}
          onUp={endDrag}
        />
      ))}
    </group>
  );
}

interface HandleEvents {
  onOver: () => void;
  onOut: () => void;
  onDown: (e: ThreeEvent<PointerEvent>) => void;
  onMove: (e: ThreeEvent<PointerEvent>) => void;
  onUp: (e: ThreeEvent<PointerEvent>) => void;
}

function FacePlane({
  face,
  roi,
  highlighted,
  dragging,
  onOver,
  onOut,
  onDown,
  onMove,
  onUp,
}: {
  face: FaceId;
  roi: WorkcellRoi;
  highlighted: boolean;
  dragging: boolean;
} & HandleEvents) {
  const size = roiSize(roi);
  const pos = faceCenter(roi, face);
  const axis = FACE_AXIS[face];
  // plane(XY 기준) → 면 법선으로 회전 + 접평면 2치수
  const rotation: [number, number, number] =
    axis === "x" ? [0, Math.PI / 2, 0] : axis === "y" ? [Math.PI / 2, 0, 0] : [0, 0, 0];
  const dims: [number, number] =
    axis === "x" ? [size[2], size[1]] : axis === "y" ? [size[0], size[2]] : [size[0], size[1]];
  return (
    <mesh
      position={pos}
      rotation={rotation}
      onPointerOver={onOver}
      onPointerOut={onOut}
      onPointerDown={onDown}
      onPointerMove={onMove}
      onPointerUp={onUp}
    >
      <planeGeometry args={dims} />
      <meshBasicMaterial
        color={VizColor.SENSOR}
        transparent
        opacity={
          dragging ? FACE_OPACITY_DRAG : highlighted ? FACE_OPACITY_HOVER : FACE_OPACITY_IDLE
        }
        depthWrite={false}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}

function AxisArrow({
  axis,
  center,
  highlighted,
  onOver,
  onOut,
  onDown,
  onMove,
  onUp,
}: {
  axis: Axis;
  center: [number, number, number];
  highlighted: boolean;
} & HandleEvents) {
  // cylinder 는 +Y 기준 — 축별 회전
  const rotation: [number, number, number] =
    axis === "x" ? [0, 0, -Math.PI / 2] : axis === "z" ? [Math.PI / 2, 0, 0] : [0, 0, 0];
  const dir = AXIS_DIR[axis];
  const shaftCenter: [number, number, number] = [
    center[0] + (dir[0] * ARROW_SHAFT_M) / 2,
    center[1] + (dir[1] * ARROW_SHAFT_M) / 2,
    center[2] + (dir[2] * ARROW_SHAFT_M) / 2,
  ];
  const tip: [number, number, number] = [
    center[0] + dir[0] * ARROW_SHAFT_M,
    center[1] + dir[1] * ARROW_SHAFT_M,
    center[2] + dir[2] * ARROW_SHAFT_M,
  ];
  const events = {
    onPointerOver: onOver,
    onPointerOut: onOut,
    onPointerDown: onDown,
    onPointerMove: onMove,
    onPointerUp: onUp,
  };
  return (
    <group renderOrder={999}>
      <mesh position={shaftCenter} rotation={rotation} {...events}>
        <cylinderGeometry
          args={[ARROW_RADIUS_M, ARROW_RADIUS_M, ARROW_SHAFT_M, 8]}
        />
        <meshBasicMaterial
          color={AXIS_COLOR[axis]}
          transparent
          opacity={highlighted ? 1.0 : 0.75}
          depthTest={false}
        />
      </mesh>
      <mesh position={tip} rotation={rotation} {...events}>
        <coneGeometry args={[ARROW_RADIUS_M * 2.6, ARROW_RADIUS_M * 6, 10]} />
        <meshBasicMaterial
          color={AXIS_COLOR[axis]}
          transparent
          opacity={highlighted ? 1.0 : 0.75}
          depthTest={false}
        />
      </mesh>
    </group>
  );
}
