/**
 * MovePreviewScenePart — MovePreviewPanel 의 scenePart (3D 표현).
 *
 * 세 요소 (전부 backend 계산 소비, 프론트는 렌더만):
 *   - 목표 마커 : 입력한 TCP pose 를 축(triad)으로. 입력창 수치 바꾸면 실시간 이동
 *     (backend 호출 없음). rpy→quat 은 intrinsic XYZ (backend `_rpy_to_quat` 과 일치).
 *   - 고스트    : plan 프레임을 재생하는 반투명 로봇. 실 로봇(live tcp_state)은 별도라
 *     안 움직임. 프레임 = 50Hz 시간등분 → 재생하면 실 속도·가감속. 배속은 시간축만.
 *   - TCP 트레이스: 프레임별 TCP 위치 폴리라인. MoveL=직선 / MoveJ(pose)=곡선.
 *     재생 프레임까지 progressive 하게 자람. feasible=False 면 끊긴 지점에 경고 마커.
 *   - 속도 점(SpeedDots): 트레이스 위에 시간 등간격(매 N프레임) 샘플점. 프레임이
 *     50Hz 등간격이라 점 사이 거리 = 그 시간 이동거리 = 속도 → 벌어진 곳=빠름(순항),
 *     좁은 곳=느림(가감속). 물리 종이테이프(ticker-tape) 원리로 가감속을 눈에 보이게.
 *     거리 등간격으로 솎으면 속도 정보가 사라지므로 반드시 시간(프레임) 등간격.
 *
 * scenePart 규약: useRobotId (RobotProvider 공급), 데이터는 movePreviewStore,
 * 패널 닫으면 함께 사라짐 (인스턴스 lifecycle).
 */
import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { useFrame } from "@react-three/fiber";
import { useRobotId } from "@/hooks/useRobotId";
import { useRobots } from "@/hooks/useRobots";
import { RobotModel } from "@/components/scene/objects/RobotModel";
import { RobotFrame } from "@/components/scene/shared/RobotFrame";
import { Frame, PolyLine, Marker } from "@/components/scene/shared/primitives";
import { VizColor } from "@/components/scene/theme/visualizationColors";
import { useMovePreviewStore } from "@/stores/movePreviewStore";

const GHOST_OPACITY = 0.35;
// 프레임은 TrajectoryRunner 의 50Hz(TRAJ_DT) 시간등분 → 1배속 = 실 로봇 속도.
const FRAME_HZ = 50;
// 속도 점 = 매 STRIDE 프레임(= STRIDE·20ms)마다 하나. 시간 등간격이라 점 간격이
// 곧 속도(가속=벌어짐/감속=좁아짐). 5 = 100ms → 순항 ~10mm·가속초입 ~2-3mm 로
// 대비가 눈에 확 들어오는 지점. 은은하게: 작고 반투명, 선은 경로로 그대로 유지.
const SPEED_DOT_STRIDE = 1;
const SPEED_DOT_RADIUS = 0.0003;
const SPEED_DOT_OPACITY = 0.7;

/** 트레이스를 시간 등간격으로 솎아 찍는 속도 점 — 간격으로 가감속을 보여준다. */
function SpeedDots({
  points,
}: {
  points: readonly (readonly [number, number, number])[];
}) {
  const dots: (readonly [number, number, number])[] = [];
  for (let i = 0; i < points.length; i += SPEED_DOT_STRIDE)
    dots.push(points[i]);
  return (
    <>
      {dots.map((p, i) => (
        // 항상 최전면 — 로봇/고스트 메시에 가려지면 가감속 리듬이 안 보인다.
        <mesh key={i} position={[p[0], p[1], p[2]]} renderOrder={999}>
          <sphereGeometry args={[SPEED_DOT_RADIUS, 12, 12]} />
          <meshBasicMaterial
            color={VizColor.PREVIEW}
            transparent
            opacity={SPEED_DOT_OPACITY}
            depthTest={false}
            depthWrite={false}
          />
        </mesh>
      ))}
    </>
  );
}

function rpyToQuat(
  rpyDeg: [number, number, number],
): [number, number, number, number] {
  const q = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(
      THREE.MathUtils.degToRad(rpyDeg[0]),
      THREE.MathUtils.degToRad(rpyDeg[1]),
      THREE.MathUtils.degToRad(rpyDeg[2]),
      "XYZ",
    ),
  );
  return [q.x, q.y, q.z, q.w];
}

export function MovePreviewScenePart() {
  const robotId = useRobotId();
  const { robots } = useRobots();
  const target = useMovePreviewStore((s) => s.targets[robotId]);
  const plan = useMovePreviewStore((s) => s.plans[robotId]);
  const speed = useMovePreviewStore((s) => s.speeds[robotId]) ?? 1;

  // 재생 상태 — token 이 바뀌면(프리뷰 버튼 클릭) 프레임 0 부터.
  const [frameIdx, setFrameIdx] = useState(0);
  const clockRef = useRef(0);
  const tokenRef = useRef<number | undefined>(undefined);

  useEffect(() => {
    if (plan && plan.token !== tokenRef.current) {
      tokenRef.current = plan.token;
      clockRef.current = 0;
      setFrameIdx(0);
    }
  }, [plan]);

  useFrame((_, delta) => {
    const frames = plan?.frames;
    if (!frames || frames.length === 0) return;
    clockRef.current += delta * speed;
    const idx = Math.min(
      frames.length - 1,
      Math.floor(clockRef.current * FRAME_HZ),
    );
    setFrameIdx((cur) => (cur === idx ? cur : idx));
  });

  const robot = robots.find((r) => r.id === robotId);
  if (!robot) return null;

  const ghostFrame =
    plan?.frames[Math.min(frameIdx, (plan?.frames.length ?? 1) - 1)];
  const tracePts = plan ? plan.tcpTrace.slice(0, frameIdx + 1) : [];
  const failPt =
    plan && !plan.feasible && plan.tcpTrace.length > 0
      ? plan.tcpTrace[plan.tcpTrace.length - 1]
      : null;

  return (
    <>
      {/* 목표 마커 + 트레이스 — robot base frame (backend 좌표 그대로). */}
      <RobotFrame>
        {target &&
          (target.useOrientation ? (
            // 목표 자세 지정 → 축 triad (자세 표현)
            <Frame
              pose={{
                position: target.position,
                quaternion: rpyToQuat(target.rpyDeg),
              }}
              size={0.05}
              label="target"
              labelColor={VizColor.TARGET}
            />
          ) : (
            // 위치만 → 점 (자세는 무관하므로 방향 표시 안 함)
            <Marker
              position={target.position}
              color={VizColor.TARGET}
              radius={0.01}
              label="target (위치만)"
            />
          ))}
        {tracePts.length >= 2 && (
          <>
            <PolyLine
              points={tracePts}
              color={VizColor.PREVIEW}
              lineWidth={2}
              overlay
            />
            <SpeedDots points={tracePts} />
          </>
        )}
        {failPt && (
          <Marker
            position={failPt}
            color={VizColor.WARNING}
            radius={0.008}
            label="도달 불가"
          />
        )}
      </RobotFrame>

      {/* 고스트 — plan 프레임 재생 (self-wrap: base_pose 내부 적용). */}
      {plan && ghostFrame && (
        <RobotModel
          robotType={robot.type}
          basePose={robot.base_pose}
          opacity={GHOST_OPACITY}
          tint={VizColor.PREVIEW}
          jointNames={plan.jointNames}
          jointAngles={ghostFrame}
        />
      )}
    </>
  );
}
