/**
 * TaskMarkersOverlay — task 가 계산한 계획 산출물(파지/적치 지점)의 3D 시각화.
 * 기능 오버레이 (topic 수명 — 마커는 패널이 닫혀도 남는 진단 도구라 scenePart 아님).
 *
 * 시각화 데이터는 그걸 계산한 쪽이 소유 (detector DETECTIONS 동형): task 가
 * PICKANDPLACE_MARKERS 를 typed 스트림으로 발행, 여기선 그대로 렌더. 발행마다
 * 전체 스냅샷(latest-wins), 새 run 시작(STATE RUNNING 전이)이면 clear.
 * 좌표는 robot base frame → <RobotFrame> 부모 transform (per-robot).
 */
import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import { Quaternion, Vector3 } from "three";
import { Text } from "@react-three/drei";
import { bridge, topicFor } from "@/api/bridge";
import { TaskStatus, Topic } from "@/api/generated/contract";
// handover(2026-07-17) 계약과 이름 충돌 → export 가 모듈 prefix 로 개명
import type {
  PickAndPlaceTaskMarker,
  PickAndPlaceTaskMarkers,
} from "@/api/generated/contract";
import type { SceneObjectProps } from "../sceneTypes";
import { RobotFrame } from "../shared/RobotFrame";
import { VizColor } from "../theme/visualizationColors";

export function TaskMarkersOverlay({ robots, focusId }: SceneObjectProps) {
  // 대상 robot = focus ?? 첫 robot. 로드 전엔 미마운트.
  const robotId = focusId ?? robots[0]?.id ?? "";
  if (!robotId) return null;
  return <MarkersForRobot robotId={robotId} />;
}

function MarkersForRobot({ robotId }: { robotId: string }) {
  const [markers, setMarkers] = useState<PickAndPlaceTaskMarker[]>([]);
  const lastStatus = useRef<string>("");

  useEffect(() => {
    const unMarkers = bridge.subscribe(
      topicFor(Topic.PICKANDPLACE_MARKERS, robotId),
      (d) => setMarkers((d as unknown as PickAndPlaceTaskMarkers).markers ?? []),
    );
    // 새 run 시작 (비-running → running 전이) = 이전 마커 무효 → clear
    const unState = bridge.subscribe(
      topicFor(Topic.PICKANDPLACE_STATE, robotId),
      (d) => {
        const status = (d as { status?: string }).status ?? "";
        if (
          status === TaskStatus.RUNNING &&
          lastStatus.current !== TaskStatus.RUNNING
        ) {
          setMarkers([]);
        }
        lastStatus.current = status;
      },
    );
    return () => {
      unMarkers();
      unState();
    };
  }, [robotId]);

  if (markers.length === 0) return null;

  // 마커는 **항상 최전면** (depthTest off + 높은 renderOrder) — 파지/적치 지점은
  // 물체·테이블 표면 그 자체라, World 배경 메시가 켜지면 마커가 메시 속에 파묻힌다
  // (2026-07-18 사용자 요구). 게임엔진/Blender 기즈모 관례: 진단 마커는 깊이를
  // 무시하고 위에 그린다 (팔이 앞을 지나도 비쳐 보이는 건 의도된 동작).
  return (
    <RobotFrame robotId={robotId}>
      {markers.map((m, i) => (
        <group key={`${m.label}:${i}`} position={m.position} renderOrder={999}>
          <mesh renderOrder={999}>
            <boxGeometry args={[0.012, 0.012, 0.012]} />
            <meshStandardMaterial
              color={VizColor.TARGET}
              depthTest={false}
              depthWrite={false}
              transparent
            />
          </mesh>
          {/* 파지 방향 (2026-07-19): approach 화살표 = 그리퍼 진입 방향 (팁이
              마커에 닿음), jaw_axis 양방향 바 = 조가 닫히는 축 = "이 두 면을
              문다". 소스 = servo.GraspFamily (refit/재플랜 시 실시간 갱신). */}
          {m.approach && <ApproachArrow dir={m.approach} />}
          {m.jaw_axis && <JawBar dir={m.jaw_axis} />}
          <Text
            position={[0, 0, 0.03]}
            fontSize={0.012}
            color={VizColor.TARGET}
            anchorX="center"
            anchorY="bottom"
            outlineWidth={0.001}
            outlineColor="#000000"
            renderOrder={1000}
            material-depthTest={false}
            material-depthWrite={false}
            material-transparent
          >
            {m.label}
          </Text>
        </group>
      ))}
    </RobotFrame>
  );
}

/** 로컬 +Y 를 dir(base 단위벡터)로 돌리는 그룹 — 화살표/바의 공통 회전. */
function AlignY({
  dir,
  children,
}: {
  dir: [number, number, number];
  children: ReactNode;
}) {
  const quat = useMemo(
    () =>
      new Quaternion().setFromUnitVectors(
        new Vector3(0, 1, 0),
        new Vector3(...dir).normalize(),
      ),
    [dir],
  );
  return <group quaternion={quat}>{children}</group>;
}

const ARROW_LEN = 0.05; // 화살대 길이 (m)
const TIP_LEN = 0.012;

/** 진입 방향 화살표 — dir 방향으로 날아와 팁이 마커(원점)에 닿는다. */
function ApproachArrow({ dir }: { dir: [number, number, number] }) {
  return (
    <AlignY dir={dir}>
      {/* +Y = dir 이므로 화살은 -Y 쪽(진입 반대편)에서 원점으로 */}
      <mesh position={[0, -TIP_LEN / 2, 0]} renderOrder={999} data-part="tip">
        <coneGeometry args={[0.004, TIP_LEN, 12]} />
        <meshStandardMaterial
          color={VizColor.PREVIEW}
          depthTest={false}
          depthWrite={false}
          transparent
        />
      </mesh>
      <mesh
        position={[0, -TIP_LEN - ARROW_LEN / 2, 0]}
        renderOrder={999}
        data-part="shaft"
      >
        <cylinderGeometry args={[0.0015, 0.0015, ARROW_LEN, 8]} />
        <meshStandardMaterial
          color={VizColor.PREVIEW}
          depthTest={false}
          depthWrite={false}
          transparent
        />
      </mesh>
    </AlignY>
  );
}

const JAW_SPAN = 0.05; // 조 축 바 전체 길이 (m) — 조 개구(≈35mm)보다 살짝 크게

/** 조 이동 축 — 마커를 관통하는 양방향 바 + 양끝 knob (조 두 팁). */
function JawBar({ dir }: { dir: [number, number, number] }) {
  return (
    <AlignY dir={dir}>
      <mesh renderOrder={999} data-part="bar">
        <cylinderGeometry args={[0.001, 0.001, JAW_SPAN, 8]} />
        <meshStandardMaterial
          color={VizColor.PREVIEW}
          depthTest={false}
          depthWrite={false}
          transparent
        />
      </mesh>
      {[JAW_SPAN / 2, -JAW_SPAN / 2].map((y) => (
        <mesh key={y} position={[0, y, 0]} renderOrder={999} data-part="knob">
          <sphereGeometry args={[0.003, 12, 12]} />
          <meshStandardMaterial
            color={VizColor.PREVIEW}
            depthTest={false}
            depthWrite={false}
            transparent
          />
        </mesh>
      ))}
    </AlignY>
  );
}
