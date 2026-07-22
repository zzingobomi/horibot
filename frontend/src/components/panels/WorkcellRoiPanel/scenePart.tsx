/**
 * WorkcellRoiScenePart — ROI 박스 3D 표현 + 직접 편집 (draft 렌더).
 *
 * 렌더 구조 (Unity BoxBoundsHandle / CAD 관례 — pnp_scenario_rework §9.1-3):
 *   - 와이어  : EdgesGeometry 12모서리. dirty(미저장) = TARGET 색으로 경고.
 *   - 반투명 면: 6개 독립 plane — **시각 전용**(볼륨을 보여줄 뿐 픽킹 안 함,
 *               raycast 무력화). depthWrite off — 뒤 포인트클라우드가 안 가려짐.
 *               조작 대상 면은 진하게 강조(핸들 노브 hover/drag 와 연동).
 *   - 핸들    : **면 중심 노브 6개** = 그 면 resize (Shift+drag = 그 축 translate).
 *               depthTest off + renderOrder 높게 → 가려진 뒷면 노브도 위에 그려져
 *               카메라 회전 없이 바로 클릭 가능(전체 면 픽킹의 관통 모호함 제거).
 *   - 중앙 3축 화살표: 박스 전체 translate (발견 가능한 이동 주 경로).
 *
 * 왜 면이 아니라 노브인가: 6면이 전부 반투명이라 한 화면 지점의 ray 가 여러 면을
 * 관통 — 호버(전파 안 끊음→먼 면 승)와 클릭(전파 끊음→가까운 면 승)이 서로 다른
 * 면을 잡아 "호버한 면과 다른 면이 움직이는" 버그가 났다. 작은 노브는 공간상
 * 분리돼 픽킹이 1:1 로 결정적 (2026-07-22 재설계).
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
// 면 핸들 노브 — 공간상 분리된 클릭 타깃 (전체 면 픽킹의 관통 모호함 제거)
const KNOB_RADIUS_M = 0.012;

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
  const visible = useWorkcellRoiStore((s) => s.visible[robotId]);
  if (!draft) return null; // ROI 미설정 — 패널의 "ROI 만들기"가 진입점
  if (visible === false) return null; // 표시 토글 off — 편집 데이터는 store 에 유지
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
      <lineSegments geometry={edges} position={center} renderOrder={998}>
        <lineBasicMaterial
          color={dirty ? VizColor.TARGET : VizColor.SENSOR}
          transparent
          opacity={0.9}
          depthTest={false}
        />
      </lineSegments>

      {/* 반투명 면 6개 = 볼륨 시각 전용 (픽킹 안 함 — 조작은 아래 노브가 담당) */}
      {FACES.map((face) => (
        <FacePlane
          key={face}
          face={face}
          roi={roi}
          highlighted={hovered === face || dragTarget === face}
          dragging={dragTarget === face}
        />
      ))}

      {/* 면 중심 노브 6개 = resize 핸들 (Shift+drag = 그 축 translate) */}
      {FACES.map((face) => (
        <FaceKnob
          key={face}
          face={face}
          roi={roi}
          highlighted={hovered === face || dragTarget === face}
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

/** 반투명 면 — 볼륨 시각 전용. raycast 무력화로 포인터 이벤트를 안 가로챈다
 * (조작은 FaceKnob 이 담당 — 면 관통 픽킹의 모호함 제거). */
function FacePlane({
  face,
  roi,
  highlighted,
  dragging,
}: {
  face: FaceId;
  roi: WorkcellRoi;
  highlighted: boolean;
  dragging: boolean;
}) {
  const size = roiSize(roi);
  const pos = faceCenter(roi, face);
  const axis = FACE_AXIS[face];
  // plane(XY 기준) → 면 법선으로 회전 + 접평면 2치수
  const rotation: [number, number, number] =
    axis === "x" ? [0, Math.PI / 2, 0] : axis === "y" ? [Math.PI / 2, 0, 0] : [0, 0, 0];
  const dims: [number, number] =
    axis === "x" ? [size[2], size[1]] : axis === "y" ? [size[0], size[2]] : [size[0], size[1]];
  return (
    <mesh position={pos} rotation={rotation} raycast={NO_RAYCAST}>
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

/** raycast noop — 이 메시는 포인터 픽킹 대상에서 제외 (three Object3D.raycast 시그니처). */
const NO_RAYCAST: THREE.Object3D["raycast"] = () => {};

/** 면 중심 클릭 노브 — 그 면 resize 핸들. depthTest off + 높은 renderOrder 로
 * 박스에 가려진 뒷면 노브도 위에 그려져 카메라 회전 없이 바로 잡을 수 있다. */
function FaceKnob({
  face,
  roi,
  highlighted,
  onOver,
  onOut,
  onDown,
  onMove,
  onUp,
}: {
  face: FaceId;
  roi: WorkcellRoi;
  highlighted: boolean;
} & HandleEvents) {
  const pos = faceCenter(roi, face);
  return (
    <mesh
      position={pos}
      renderOrder={999}
      onPointerOver={(e) => {
        e.stopPropagation();
        onOver();
      }}
      onPointerOut={(e) => {
        e.stopPropagation();
        onOut();
      }}
      onPointerDown={onDown}
      onPointerMove={onMove}
      onPointerUp={onUp}
    >
      <sphereGeometry args={[KNOB_RADIUS_M, 16, 12]} />
      <meshBasicMaterial
        color={VizColor.SENSOR}
        transparent
        opacity={highlighted ? 1.0 : 0.85}
        depthTest={false}
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
    onPointerOver: (e: ThreeEvent<PointerEvent>) => {
      e.stopPropagation();
      onOver();
    },
    onPointerOut: (e: ThreeEvent<PointerEvent>) => {
      e.stopPropagation();
      onOut();
    },
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
