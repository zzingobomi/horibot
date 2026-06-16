/**
 * ServoTCP — 절대 TCP target 직접 IK + publish (planner 우회 chase 패턴).
 *
 * gamepad 없는 자리 debugging 용. orientation 토글 ON 시 6DOF (Euler XYZ deg).
 * 5DOF robot 은 quaternion 필드 무시 (server side position-only IK fallback).
 */
import { useCallback, useEffect, useState } from "react";
import { Euler, Quaternion } from "three";
import type { Vector3Tuple } from "three";
import { PanelButton } from "@/components/shared/PanelButton";
import { useService } from "@/framework";
import { ServiceKey } from "@/constants/topics";
import { mmToMVec3, mToMmVec3 } from "@/lib/robot/utils";

const AXES = ["X", "Y", "Z"] as const;
const ANG_AXES = ["RX", "RY", "RZ"] as const;

function quatToEulerDeg(q: number[]): Vector3Tuple {
  const quat = new Quaternion(q[0], q[1], q[2], q[3]);
  const euler = new Euler().setFromQuaternion(quat, "XYZ");
  return [
    (euler.x * 180) / Math.PI,
    (euler.y * 180) / Math.PI,
    (euler.z * 180) / Math.PI,
  ];
}

function eulerDegToQuat(e: Vector3Tuple): number[] {
  const euler = new Euler(
    (e[0] * Math.PI) / 180,
    (e[1] * Math.PI) / 180,
    (e[2] * Math.PI) / 180,
    "XYZ",
  );
  const q = new Quaternion().setFromEuler(euler);
  return [q.x, q.y, q.z, q.w];
}

export function ServoTCPControl() {
  const tcpSvc = useService(ServiceKey.MOTION_GET_TCP);
  const servoTCP = useService(ServiceKey.MOTION_SERVO_TCP);
  const tcpPose = tcpSvc.data;

  const [posMm, setPosMm] = useState<Vector3Tuple>([0, 0, 0]);
  const [eulerDeg, setEulerDeg] = useState<Vector3Tuple>([0, 0, 0]);
  const [useOrientation, setUseOrientation] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!tcpPose) return;
    const newPos = mToMmVec3(tcpPose.position) as Vector3Tuple;
    const newEuler = quatToEulerDeg(tcpPose.quaternion);
    // eslint-disable-next-line react-hooks/set-state-in-effect
    setPosMm(newPos);
    setEulerDeg(newEuler);
  }, [tcpPose]);

  const handleServo = useCallback(async () => {
    setError(null);
    const positionM = mmToMVec3(posMm);
    const payload: {
      position: number[];
      quaternion?: number[] | null;
    } = { position: positionM };
    if (useOrientation) payload.quaternion = eulerDegToQuat(eulerDeg);
    const res = await servoTCP.call(payload);
    if (!res.success) setError(res.message || "ServoTcp 실패");
  }, [posMm, eulerDeg, useOrientation, servoTCP]);

  return (
    <div className="flex flex-col gap-3">
      {tcpPose && (
        <div className="rounded bg-zinc-900/60 border border-zinc-800/60 px-2.5 py-2 font-mono">
          <p className="text-[9px] uppercase tracking-widest text-zinc-600 mb-1.5">
            현재 TCP (mm / deg)
          </p>
          <div className="grid grid-cols-3 gap-2 text-[10px] tabular-nums">
            {mToMmVec3(tcpPose.position).map((v, i) => (
              <div key={AXES[i]}>
                <span className="text-zinc-500">{AXES[i]}: </span>
                <span className="text-zinc-300">{v.toFixed(1)}</span>
              </div>
            ))}
          </div>
          <div className="grid grid-cols-3 gap-2 text-[10px] tabular-nums mt-1">
            {quatToEulerDeg(tcpPose.quaternion).map((v, i) => (
              <div key={ANG_AXES[i]}>
                <span className="text-zinc-500">{ANG_AXES[i]}: </span>
                <span className="text-zinc-300">{v.toFixed(1)}</span>
              </div>
            ))}
          </div>
        </div>
      )}

      <div className="flex flex-col gap-1.5">
        <p className="text-[9px] uppercase tracking-widest text-zinc-600 font-mono">
          목표 위치 (mm)
        </p>
        <div className="grid grid-cols-3 gap-2">
          {AXES.map((ax, i) => (
            <div key={ax} className="flex flex-col gap-1">
              <span className="text-[10px] text-zinc-500 font-mono">{ax}</span>
              <input
                type="number"
                step={1}
                value={posMm[i]}
                onChange={(e) => {
                  const num = parseFloat(e.target.value);
                  if (!isNaN(num))
                    setPosMm((prev) => {
                      const next: Vector3Tuple = [...prev];
                      next[i] = num;
                      return next;
                    });
                }}
                className="h-7 w-full px-1.5 text-[11px] text-right text-blue-400 tabular-nums font-mono bg-zinc-900 border border-zinc-800 rounded focus:outline-none focus:border-blue-500/60"
              />
            </div>
          ))}
        </div>
      </div>

      <label className="flex items-center gap-2 text-[10px] text-zinc-400 font-mono cursor-pointer">
        <input
          type="checkbox"
          checked={useOrientation}
          onChange={(e) => setUseOrientation(e.target.checked)}
          className="accent-blue-500"
        />
        <span>orientation 사용 (6DOF) — 5DOF robot 은 무시</span>
      </label>

      {useOrientation && (
        <div className="flex flex-col gap-1.5">
          <p className="text-[9px] uppercase tracking-widest text-zinc-600 font-mono">
            목표 자세 (Euler XYZ deg)
          </p>
          <div className="grid grid-cols-3 gap-2">
            {ANG_AXES.map((ax, i) => (
              <div key={ax} className="flex flex-col gap-1">
                <span className="text-[10px] text-zinc-500 font-mono">{ax}</span>
                <input
                  type="number"
                  step={1}
                  value={eulerDeg[i]}
                  onChange={(e) => {
                    const num = parseFloat(e.target.value);
                    if (!isNaN(num))
                      setEulerDeg((prev) => {
                        const next: Vector3Tuple = [...prev];
                        next[i] = num;
                        return next;
                      });
                  }}
                  className="h-7 w-full px-1.5 text-[11px] text-right text-blue-400 tabular-nums font-mono bg-zinc-900 border border-zinc-800 rounded focus:outline-none focus:border-blue-500/60"
                />
              </div>
            ))}
          </div>
        </div>
      )}

      {error && <p className="text-[10px] font-mono text-red-400">{error}</p>}

      <div className="flex gap-2">
        <PanelButton
          variant="outline"
          onClick={() => void tcpSvc.call({})}
          disabled={tcpSvc.pending}
          className="flex-1"
        >
          {tcpSvc.pending ? "읽는 중…" : "TCP 동기화"}
        </PanelButton>
        <PanelButton
          variant="primary"
          onClick={() => void handleServo()}
          disabled={servoTCP.pending}
          className="flex-1"
        >
          {servoTCP.pending ? "전송 중…" : "Servo"}
        </PanelButton>
      </div>
    </div>
  );
}
