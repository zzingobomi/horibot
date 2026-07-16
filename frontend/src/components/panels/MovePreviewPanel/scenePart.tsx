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

  const ghostFrame = plan?.frames[Math.min(frameIdx, (plan?.frames.length ?? 1) - 1)];
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
          <PolyLine points={tracePts} color={VizColor.PREVIEW} lineWidth={2} />
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
