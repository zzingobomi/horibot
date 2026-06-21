/**
 * Robot State + 기본 컨트롤 — 모든 mode 공통. joint state 읽기, Torque ON/OFF,
 * Home 이동, (확장 시) 5DOF 슬라이더 jog 까지 한 패널에서 끝.
 *
 * 캘리브레이션 모드의 수동 자세 캡처 워크플로우 ("토크 끄고 손으로 잡고 캡처")
 * 는 이 패널에서 처리. 카메라 overlay 자리에 컨트롤 박지 않음.
 */
import { useCallback, useMemo, useState } from "react";
import { useParams } from "react-router-dom";
import { Activity, ChevronDown, ChevronUp } from "lucide-react";
import * as SliderPrimitive from "@radix-ui/react-slider";
import type { IDockviewPanelProps } from "dockview";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { useJointOffsetsRad } from "@/hooks/useCalibrationResults";
import { useSceneStore } from "@/domain/stores/scene";
import { bridge } from "@/api/bridge";
import { PanelShell } from "@/components/shared/PanelShell";
import { PanelButton } from "@/components/shared/PanelButton";
import { Section } from "@/components/shared/Section";
import { loadPose } from "@/lib/robot/robotPoses";
import { formatDeg, rawToUrdfDeg } from "@/lib/robot/utils";
import { useArmJoints } from "@/lib/robot/config";
import type { Joint } from "@/types/motor";

const EMPTY_JOINTS: Joint[] = [];

export function RobotStatePanel(props: IDockviewPanelProps<object>) {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const joints = useTopic(Topic.MOTOR_STATE_JOINT, robotId)?.joints ?? EMPTY_JOINTS;
  const jointOffsetsRad = useJointOffsetsRad(robotId);
  const tcpPos = useSceneStore((s) => s.tcpPos);
  // robots.yaml 의 arm motors SSOT — DOF 자리 hardcode 폐기 (5DOF/6DOF 일반화).
  const armJoints = useArmJoints(robotId);

  const cfgSvc = useService(ServiceKey.MOTOR_GET_CONFIG, robotId);
  const configs = cfgSvc.data?.motors ?? [];
  const torqueEnabled = cfgSvc.data?.torque_enabled ?? false;
  const enableSvc = useService(ServiceKey.MOTOR_ENABLE, robotId);
  const moveJ = useService(ServiceKey.MOTION_MOVE_J, robotId);

  const [jogOpen, setJogOpen] = useState(false);
  const [cmdPositions, setCmdPositions] = useState<Record<number, number>>({});

  // URDF rad (joint_offset 적용된 kinematic frame). 모든 frontend 표시 frame SSOT.
  const jointAngles = useMemo(() => {
    if (!armJoints.length) return [];
    return armJoints.map((cfg) => {
      const j = joints.find((x) => x.id === cfg.id);
      if (!j) return 0;
      const baseRad =
        j.degree !== undefined
          ? (j.degree * Math.PI) / 180
          : j.position !== undefined
            ? ((j.position - 2048) / 4095) * 2 * Math.PI
            : 0;
      return baseRad + (jointOffsetsRad[cfg.id] ?? 0);
    });
  }, [armJoints, joints, jointOffsetsRad]);

  const handleJointCmd = useCallback(
    (id: number, position: number) => {
      setCmdPositions((prev) => ({ ...prev, [id]: position }));
      bridge.publish(
        Topic.MOTOR_CMD_JOINT,
        {
          timestamp: Date.now() / 1000,
          joints: [{ id, position }],
        },
        robotId,
      );
    },
    [robotId],
  );

  const syncAll = useCallback(() => {
    setCmdPositions(Object.fromEntries(joints.map((j) => [j.id, j.position])));
  }, [joints]);

  const goHome = useCallback(async () => {
    const pose = await loadPose(robotId, "home");
    await moveJ.call({ joints: pose });
  }, [robotId, moveJ]);

  const goRest = useCallback(async () => {
    const pose = await loadPose(robotId, "rest");
    await moveJ.call({ joints: pose });
  }, [robotId, moveJ]);

  const toggleTorque = useCallback(async () => {
    const next = !torqueEnabled;
    const res = await enableSvc.call({ enable: next });
    if (res.success) await cfgSvc.call({});
  }, [torqueEnabled, enableSvc, cfgSvc]);

  return (
    <PanelShell
      icon={<Activity className="w-3.5 h-3.5" />}
      title="Robot State"
      panelId={props.api.id}
      api={props.api}
      expandedHeight={400}
    >
      <Section label="Control">
        <div className="flex flex-col gap-2">
          <PanelButton
            variant={torqueEnabled ? "primary" : "danger"}
            onClick={() => void toggleTorque()}
          >
            Torque {torqueEnabled ? "ON" : "OFF"}
          </PanelButton>
          <div className="flex items-center gap-2">
            <PanelButton
              variant="outline"
              onClick={() => void goHome()}
              className="flex-1"
            >
              Home
            </PanelButton>
            <PanelButton
              variant="outline"
              onClick={() => void goRest()}
              className="flex-1"
            >
              Rest
            </PanelButton>
          </div>
        </div>
      </Section>

      <Section label="Joint Angles">
        <div className="font-mono text-[11px] space-y-1">
          {jointAngles.map((rad, i) => (
            <div key={i} className="flex items-center gap-2">
              <span className="text-zinc-600 w-4">J{i + 1}</span>
              <div className="flex-1 h-0.5 bg-zinc-800 rounded overflow-hidden">
                <div
                  className="h-full bg-blue-500/70 rounded transition-all duration-100"
                  style={{
                    width: `${((rad + Math.PI) / (2 * Math.PI)) * 100}%`,
                  }}
                />
              </div>
              <span className="text-zinc-300 tabular-nums w-14 text-right">
                {((rad * 180) / Math.PI).toFixed(1)}°
              </span>
            </div>
          ))}
        </div>
      </Section>

      <Section label="TCP Position">
        {tcpPos ? (
          <div className="font-mono text-[11px] space-y-1">
            {(["x", "y", "z"] as const).map((axis, i) => (
              <div key={axis} className="flex justify-between items-center">
                <span className="text-zinc-500">{axis.toUpperCase()}</span>
                <span className="text-emerald-400 tabular-nums">
                  {tcpPos[i].toFixed(4)}
                  <span className="text-zinc-600 ml-1">m</span>
                </span>
              </div>
            ))}
          </div>
        ) : (
          <p className="text-[11px] text-zinc-600 font-mono">No robot loaded</p>
        )}
      </Section>

      <Section label="Jog">
        <div className="flex flex-col gap-2">
          <div className="flex items-center justify-between">
            <button
              onClick={() => setJogOpen((v) => !v)}
              className="flex items-center gap-1.5 text-[10px] font-mono uppercase tracking-wide text-zinc-500 hover:text-zinc-200 transition-colors"
            >
              {jogOpen ? (
                <ChevronDown className="w-3 h-3" />
              ) : (
                <ChevronUp className="w-3 h-3 rotate-180" />
              )}
              {jogOpen ? "접기" : "슬라이더 펼치기"}
            </button>
            {jogOpen && (
              <PanelButton
                variant="ghost"
                onClick={syncAll}
                className="!px-2 !py-0.5 !text-[10px]"
              >
                Sync
              </PanelButton>
            )}
          </div>

          {jogOpen && (
            <div className="flex flex-col gap-1">
              {joints.length === 0 ? (
                <p className="py-1 text-center text-[11px] text-zinc-500 font-mono">
                  모터 연결 대기 중...
                </p>
              ) : (
                <>
                  <div className="flex items-center gap-2 font-mono text-[9px] uppercase tracking-wide text-zinc-600">
                    <span className="w-9 shrink-0" />
                    <span className="flex-1" />
                    <span className="w-11 text-right text-blue-400 shrink-0">
                      cmd
                    </span>
                    <span className="w-11 text-right shrink-0">act</span>
                  </div>
                  {joints.map((joint) => {
                    const cfg = configs.find((c) => c.id === joint.id);
                    return (
                      <JogSliderRow
                        key={joint.id}
                        joint={joint}
                        cmdPosition={cmdPositions[joint.id] ?? joint.position}
                        limitMin={cfg?.limit.min ?? 0}
                        limitMax={cfg?.limit.max ?? 4095}
                        offsetRad={jointOffsetsRad[joint.id] ?? 0}
                        onValueChange={handleJointCmd}
                      />
                    );
                  })}
                </>
              )}
            </div>
          )}
        </div>
      </Section>
    </PanelShell>
  );
}

function shortName(name: string): string {
  if (name.startsWith("joint")) return "J" + name.slice(5);
  if (name === "gripper_joint" || name === "gripper") return "Grip";
  return name;
}

function JogSliderRow({
  joint,
  cmdPosition,
  limitMin,
  limitMax,
  offsetRad,
  onValueChange,
}: {
  joint: Joint;
  cmdPosition: number;
  limitMin: number;
  limitMax: number;
  /** kinematic frame 변환 (URDF degree = rawToDeg + offset_deg). default 0. */
  offsetRad: number;
  onValueChange: (id: number, position: number) => void;
}) {
  const toPercent = (val: number) =>
    ((val - limitMin) / (limitMax - limitMin)) * 100;
  const isLagging = Math.abs(cmdPosition - joint.position) > 50;

  return (
    <div className="flex items-center gap-2 font-mono text-[10px] tabular-nums">
      <span
        className="w-9 text-zinc-300 shrink-0 truncate"
        title={joint.name}
      >
        {shortName(joint.name)}
      </span>

      <SliderPrimitive.Root
        className="relative flex items-center select-none touch-none flex-1 h-4"
        min={limitMin}
        max={limitMax}
        step={1}
        value={[cmdPosition]}
        onValueChange={([v]: number[]) => onValueChange(joint.id, v)}
      >
        <SliderPrimitive.Track className="relative h-1 w-full grow rounded-full bg-zinc-800">
          <SliderPrimitive.Range className="absolute h-full rounded-full bg-blue-500/40" />
          <div
            className="absolute top-1/2 -translate-y-1/2 -translate-x-1/2 w-0.5 h-2.5 rounded-full bg-orange-400 pointer-events-none transition-[left] duration-75"
            style={{ left: `${toPercent(joint.position)}%` }}
          />
        </SliderPrimitive.Track>
        <SliderPrimitive.Thumb className="block h-3 w-3 rounded-full border border-blue-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-blue-400" />
      </SliderPrimitive.Root>

      <span className="w-11 text-right text-blue-400 shrink-0">
        {formatDeg(rawToUrdfDeg(cmdPosition, offsetRad))}°
      </span>
      <span
        className={`w-11 text-right shrink-0 ${
          isLagging ? "text-orange-400" : "text-zinc-500"
        }`}
      >
        {formatDeg(rawToUrdfDeg(joint.position, offsetRad))}°
      </span>
    </div>
  );
}
