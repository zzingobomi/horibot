/**
 * Multi-robot URDF mount — robots.yaml 의 N robot 동시 마운트.
 *
 * robot 상태 시각화(joint + TCP frame)는 robot 마다 자기 Motion.Stream.TCP_STATE
 * 를 구독 (per-robot). backend 가 per-robot stream 을 발행하고 bootstrap 이
 * robots 목록 전체를 store 에 채우므로, robot_id 로 읽기만 하면 N=2+ 협동
 * 자리에서 각자 독립으로 움직인다. (옛 "focus robot 의 stream 하나를 전원에
 * 적용" 임시 호환은 Tasks(focus=null) multi-robot 에서 충돌 — CLAUDE.md §4
 * 결정 3 의 'dict 화' 시점 도래로 제거.)
 */
import { useMemo } from "react";
import { RobotModel } from "./RobotModel";
import { AxisFrame } from "./AxisFrame";
import { robotBaseMatrix, poseToWorldMatrix } from "./transforms";
import { useStream } from "@/framework";
import { Topic } from "@/api/generated/contract";
import type { RobotInfo } from "@/api/generated/contract";

export interface RobotLayerProps {
  robots: RobotInfo[];
  /** focus robot id — null = 모두 동등. */
  focusId?: string | null;
  onLinksLoaded?: (linkNames: string[]) => void;
  dimOpacity?: number;
  showRobot?: boolean;
  /** TCP 좌표축 표시 (Scene options.showTCPFrame). focus robot 만 (dim robot 은 잡음). */
  showTcpFrame?: boolean;
}

const EMPTY_NAMES: string[] = [];
const EMPTY_JOINTS: number[] = [];

interface RobotItemProps {
  robot: RobotInfo;
  opacity: number;
  visible: boolean;
  showTcpFrame: boolean;
  onLinksLoaded?: (linkNames: string[]) => void;
}

/** robot 1대 — 자기 robot_id 의 TCP_STATE 구독 + URDF/TCP frame 렌더. */
function RobotItem({
  robot,
  opacity,
  visible,
  showTcpFrame,
  onLinksLoaded,
}: RobotItemProps) {
  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId: robot.id });
  // parallel arrays — backend Motion 이 joint_names + joints 를 same order 로 발행.
  // stream 미도착(motion 모듈 안 뜬 robot 등) 시 빈 배열 → URDF 기본 pose.
  const jointNames = tcp.value?.joint_names ?? EMPTY_NAMES;
  const jointAngles = tcp.value?.joints ?? EMPTY_JOINTS;

  // TCP pose = backend corrected FK (SSOT — frontend 자체 FK 박지 X).
  const baseMatrix = useMemo(() => robotBaseMatrix(robot.base_pose), [robot.base_pose]);
  const tcpMatrix = useMemo(() => {
    if (!tcp.value) return null;
    return poseToWorldMatrix(baseMatrix, tcp.value.position, tcp.value.quaternion);
  }, [tcp.value, baseMatrix]);

  return (
    <>
      <RobotModel
        robotType={robot.type}
        basePose={robot.base_pose}
        opacity={opacity}
        jointNames={jointNames}
        jointAngles={jointAngles}
        visible={visible}
        onLinksLoaded={onLinksLoaded}
      />
      {showTcpFrame && tcpMatrix && (
        <AxisFrame matrix={tcpMatrix} size={0.04} label="TCP" labelColor="#ffcc44" />
      )}
    </>
  );
}

export function RobotLayer({
  robots,
  focusId = null,
  onLinksLoaded,
  dimOpacity = 0.25,
  showRobot = true,
  showTcpFrame = true,
}: RobotLayerProps) {
  return (
    <>
      {robots.map((r) => {
        const isFocus = focusId === null || r.id === focusId;
        const isCallbackTarget =
          focusId === r.id || (focusId === null && r.id === robots[0]?.id);

        return (
          <RobotItem
            key={r.id}
            robot={r}
            opacity={isFocus ? 1.0 : dimOpacity}
            visible={showRobot}
            showTcpFrame={showTcpFrame && isFocus}
            onLinksLoaded={isCallbackTarget ? onLinksLoaded : undefined}
          />
        );
      })}
    </>
  );
}
