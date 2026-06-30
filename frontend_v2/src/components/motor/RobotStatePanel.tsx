/**
 * Robot 상태 표시 — connection / torque state / joint table / TCP position.
 *
 * frontend_v2 first cut — 옛 RobotStatePanel.tsx 의 simplified carry over.
 * 박지 X: dockview / radix / lucide / useArmJoints / useJointOffsetsRad /
 * loadPose / Home button (Step E+ 박힐 때).
 *
 * Wire:
 *   - Motion.Stream.TCP_STATE — joints rad + position + quaternion
 *   - Motor.Stream.RAW_STATE — positions_raw (debug)
 *   - Motor.Event.TORQUE_CHANGED — torque state reactive
 *   - Motor.Service.SET_TORQUE — torque toggle
 *   - Motor.Service.GET_TOPOLOGY — arm joint count
 */
import { useService, useStream, useCapability, useTopic, useBridgeConnected } from "@/framework";
import {
  MotorKind,
  ServiceKey,
  Topic,
  type TorqueChanged,
} from "@/api/generated/contract";

interface RobotStatePanelProps {
  robotId: string;
}

export function RobotStatePanel({ robotId }: RobotStatePanelProps) {
  const connected = useBridgeConnected();
  const cap = useCapability(ServiceKey.MOTOR_GET_TOPOLOGY, { robotId });
  const armMotors = (cap.value?.motors ?? [])
    .filter((m) => m.kind === MotorKind.JOINT)
    .sort((a, b) => a.id - b.id);

  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId });
  const raw = useStream(Topic.MOTOR_RAW_STATE, { robotId });

  // torque state — TORQUE_CHANGED event reactive. 초기값 unknown (backend 박지 X).
  const torqueEvt = useTopic(Topic.MOTOR_TORQUE_CHANGED, robotId) as
    | TorqueChanged
    | null;
  const torqueEnabled = torqueEvt?.enabled ?? null; // null = unknown

  const setTorque = useService(ServiceKey.MOTOR_SET_TORQUE, robotId);

  const handleToggle = () => {
    if (torqueEnabled === null) return;
    void setTorque.call({ enabled: !torqueEnabled });
  };

  const jointRads = tcp.value?.joints ?? [];
  const rawPositions = raw.value?.positions_raw ?? [];
  const tcpPos = tcp.value?.position ?? null;

  return (
    <div className="flex flex-col gap-3 font-mono text-zinc-300">
      {/* Connection + Torque badge */}
      <div className="flex items-center gap-2">
        <span
          className={`px-1.5 py-0.5 rounded text-[10px] uppercase ${
            connected
              ? "bg-emerald-500/20 text-emerald-300"
              : "bg-red-500/20 text-red-300"
          }`}
        >
          {connected ? "connected" : "disconnected"}
        </span>
        <span
          className={`px-1.5 py-0.5 rounded text-[10px] uppercase ${
            torqueEnabled === true
              ? "bg-emerald-500/20 text-emerald-300"
              : torqueEnabled === false
                ? "bg-zinc-700 text-zinc-400"
                : "bg-zinc-800 text-zinc-600"
          }`}
        >
          torque {torqueEnabled === null ? "?" : torqueEnabled ? "on" : "off"}
        </span>
        {tcp.stale && (
          <span className="px-1.5 py-0.5 rounded text-[10px] uppercase bg-amber-500/20 text-amber-300">
            stale {tcp.lagMs.toFixed(0)}ms
          </span>
        )}
      </div>

      {/* Torque toggle */}
      <button
        onClick={handleToggle}
        disabled={torqueEnabled === null || setTorque.pending}
        className="h-7 rounded border border-zinc-700 bg-zinc-900 text-[11px] uppercase tracking-wide hover:bg-zinc-800 disabled:opacity-40 disabled:cursor-not-allowed"
      >
        {torqueEnabled ? "torque off" : "torque on"}
      </button>

      {/* Joint table */}
      <div>
        <div className="text-[9px] uppercase tracking-wide text-zinc-500 mb-1">
          joints (arm)
        </div>
        <div className="flex flex-col gap-0.5">
          <div className="grid grid-cols-3 gap-2 text-[9px] text-zinc-600 uppercase tracking-wide">
            <div>name</div>
            <div className="text-right">deg</div>
            <div className="text-right">raw</div>
          </div>
          {armMotors.map((m, i) => (
            <div
              key={m.id}
              className="grid grid-cols-3 gap-2 text-[11px] tabular-nums"
            >
              <div className="text-zinc-300">J{i + 1}</div>
              <div className="text-right text-emerald-300">
                {jointRads[i] !== undefined
                  ? ((jointRads[i] * 180) / Math.PI).toFixed(1)
                  : "—"}
              </div>
              <div className="text-right text-zinc-500">
                {rawPositions[i] !== undefined ? rawPositions[i] : "—"}
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* TCP position */}
      <div>
        <div className="text-[9px] uppercase tracking-wide text-zinc-500 mb-1">
          tcp (m)
        </div>
        {tcpPos ? (
          <div className="grid grid-cols-3 gap-2 text-[11px] tabular-nums">
            <div>
              <span className="text-zinc-500">x </span>
              <span className="text-emerald-300">{tcpPos[0].toFixed(3)}</span>
            </div>
            <div>
              <span className="text-zinc-500">y </span>
              <span className="text-emerald-300">{tcpPos[1].toFixed(3)}</span>
            </div>
            <div>
              <span className="text-zinc-500">z </span>
              <span className="text-emerald-300">{tcpPos[2].toFixed(3)}</span>
            </div>
          </div>
        ) : (
          <div className="text-[10px] text-zinc-600">no tcp data</div>
        )}
      </div>
    </div>
  );
}
