import { useCallback, useEffect, useState } from "react";
import { ChevronDown, ChevronUp, RefreshCw } from "lucide-react";
import { Button } from "@/components/ui/button";
import { useService } from "@/framework";
import { ServiceKey } from "@/constants/topics";
import type { Vec3 } from "@/types/motion";

const STEPS = [0.001, 0.005, 0.01] as const;
type Step = (typeof STEPS)[number];

const STEP_LABELS: Record<Step, string> = {
  0.001: "1mm",
  0.005: "5mm",
  0.01: "10mm",
};

const AXES = ["x", "y", "z"] as const;
type Axis = (typeof AXES)[number];
const AXIS_INDEX: Record<Axis, number> = { x: 0, y: 1, z: 2 };

export function MoveTCPControl() {
  const tcpSvc = useService(ServiceKey.MOTION_GET_TCP);
  const moveTCP = useService(ServiceKey.MOTION_MOVE_TCP);
  const tcpPose = tcpSvc.data;

  const [step, setStep] = useState<Step>(0.005);
  const [pos, setPos] = useState<Vec3>([0, 0, 0]);

  // 서버 TCP → 로컬 pos 동기 (optimistic update rollback 자리 보존).
  useEffect(() => {
    if (tcpPose) {
      const [x, y, z] = tcpPose.position;
      // eslint-disable-next-line react-hooks/set-state-in-effect
      setPos([x, y, z]);
    }
  }, [tcpPose]);

  const doStep = useCallback(
    async (axis: Axis, direction: 1 | -1) => {
      const prev = pos;
      const next: Vec3 = [...pos];
      next[AXIS_INDEX[axis]] += step * direction;
      setPos(next);
      const res = await moveTCP.call({ position: next });
      if (!res.success) setPos(prev);
    },
    [pos, step, moveTCP],
  );

  return (
    <div className="flex flex-col gap-3">
      <div className="flex gap-1">
        {STEPS.map((s) => (
          <Button
            key={s}
            size="sm"
            variant={step === s ? "default" : "outline"}
            className="flex-1 text-xs h-7"
            onClick={() => setStep(s)}
          >
            {STEP_LABELS[s]}
          </Button>
        ))}
      </div>

      <div className="flex flex-col gap-1">
        {AXES.map((axis, i) => (
          <div key={axis} className="flex items-center gap-2">
            <span className="w-4 text-xs font-mono text-muted-foreground uppercase">
              {axis}
            </span>
            <Button
              size="sm"
              variant="outline"
              className="h-7 w-7 p-0"
              onClick={() => void doStep(axis, -1)}
              disabled={!tcpPose || moveTCP.pending}
            >
              <ChevronDown className="h-3 w-3" />
            </Button>
            <span className="flex-1 text-center font-mono text-xs tabular-nums">
              {(pos[i] * 1000).toFixed(1)} mm
            </span>
            <Button
              size="sm"
              variant="outline"
              className="h-7 w-7 p-0"
              onClick={() => void doStep(axis, 1)}
              disabled={!tcpPose || moveTCP.pending}
            >
              <ChevronUp className="h-3 w-3" />
            </Button>
          </div>
        ))}
      </div>

      <Button
        size="sm"
        variant="outline"
        className="gap-1"
        onClick={() => void tcpSvc.call({})}
        disabled={tcpSvc.pending}
      >
        <RefreshCw className="h-3 w-3" />
        현재 위치 동기화
      </Button>
    </div>
  );
}
