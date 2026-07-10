/**
 * RobotStatePanel — dockview 등록 패널 (robotState). connection / torque /
 * joint table / TCP position.
 *
 * 패널이라 router 의존(useParams)을 자체 흡수 (registry 는 순수 유지). 패널 = 폴더
 * (index.tsx), 폴더-per-panel 통일 (frontend_v2.md §4.1) — 내부 분할(Control/Status)
 * 필요 시 이 폴더에 서브 컴포넌트 추가.
 *
 * Wire:
 *   - Motion.Stream.TCP_STATE — joints rad + position + quaternion
 *   - Motor.Stream.RAW_STATE — positions_raw (debug, 20Hz kinematic)
 *   - Motor.Stream.STATE — torque_enabled (5Hz driver control state, 초기 latch)
 *   - Motor.Service.SET_TORQUE — torque toggle
 *   - Motor.Service.GET_TOPOLOGY — arm joint count
 */
import {
  useService,
  useStream,
  useCapability,
  useBridgeConnected,
} from "@/framework";
import { Button } from "@/components/ui/button";
import { MotorKind, ServiceKey, Topic } from "@/api/generated/contract";
import { useRobotId } from "@/hooks/useRobotId";

export function RobotStatePanel() {
  const robotId = useRobotId();

  const connected = useBridgeConnected();
  const cap = useCapability(ServiceKey.MOTOR_GET_TOPOLOGY, { robotId });
  const armMotors = (cap.value?.motors ?? [])
    .filter((m) => m.kind === MotorKind.JOINT)
    .sort((a, b) => a.id - b.id);

  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId });
  const raw = useStream(Topic.MOTOR_RAW_STATE, { robotId });

  // torque source = Motor.Stream.STATE (5Hz driver-state stream). event 가 아니라
  // stream 이라 mount 직후 첫 frame 오면 즉시 활성화 — event chicken-and-egg 해소.
  const driverState = useStream(Topic.MOTOR_STATE, { robotId });
  const torqueEnabled = driverState.value?.torque_enabled ?? null;

  const setTorque = useService(ServiceKey.MOTOR_SET_TORQUE, robotId);

  const handleToggle = () => {
    if (torqueEnabled === null) return;
    void setTorque.call({ enabled: !torqueEnabled });
  };

  const jointRads = tcp.value?.joints ?? [];
  const rawPositions = raw.value?.positions_raw ?? [];
  const tcpPos = tcp.value?.position ?? null;

  return (
    <div className="h-full overflow-y-auto p-3 flex flex-col gap-3 font-mono text-foreground">
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
                ? "bg-muted text-muted-foreground"
                : "bg-muted/50 text-muted-foreground/60"
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
      <Button
        variant="outline"
        size="sm"
        onClick={handleToggle}
        disabled={torqueEnabled === null || setTorque.pending}
        className="font-mono uppercase tracking-wide"
      >
        {torqueEnabled ? "torque off" : "torque on"}
      </Button>

      {/* Joint table */}
      <div>
        <div className="text-[9px] uppercase tracking-wide text-muted-foreground mb-1">
          joints (arm)
        </div>
        <div className="flex flex-col gap-0.5">
          <div className="grid grid-cols-3 gap-2 text-[9px] text-muted-foreground uppercase tracking-wide">
            <div>name</div>
            <div className="text-right">deg</div>
            <div className="text-right">raw</div>
          </div>
          {armMotors.map((m, i) => (
            <div
              key={m.id}
              className="grid grid-cols-3 gap-2 text-[11px] tabular-nums"
            >
              <div className="text-foreground">J{i + 1}</div>
              <div className="text-right text-emerald-300">
                {jointRads[i] !== undefined
                  ? ((jointRads[i] * 180) / Math.PI).toFixed(1)
                  : "—"}
              </div>
              <div className="text-right text-muted-foreground">
                {rawPositions[i] !== undefined ? rawPositions[i] : "—"}
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* TCP position */}
      <div>
        <div className="text-[9px] uppercase tracking-wide text-muted-foreground mb-1">
          tcp (m)
        </div>
        {tcpPos ? (
          <div className="grid grid-cols-3 gap-2 text-[11px] tabular-nums">
            <div>
              <span className="text-muted-foreground">x </span>
              <span className="text-emerald-300">{tcpPos[0].toFixed(3)}</span>
            </div>
            <div>
              <span className="text-muted-foreground">y </span>
              <span className="text-emerald-300">{tcpPos[1].toFixed(3)}</span>
            </div>
            <div>
              <span className="text-muted-foreground">z </span>
              <span className="text-emerald-300">{tcpPos[2].toFixed(3)}</span>
            </div>
          </div>
        ) : (
          <div className="text-[10px] text-muted-foreground">no tcp data</div>
        )}
      </div>
    </div>
  );
}
