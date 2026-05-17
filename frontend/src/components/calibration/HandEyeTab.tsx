import { useState } from "react";
import { CameraFeed } from "@/components/camera/CameraFeed";
import { Button } from "@/components/ui/button";
import { MoveTCPControl } from "@/components/robot/MoveTCPControl";
import { useJointControl } from "@/hooks/useJointControl";
import { useMotion } from "@/hooks/useMotion";
import type { HandEyePreview, PoseMeta } from "./types";
import { HandEyePoseList } from "./HandEyePoseList";
import { CheckerboardOverlay } from "./CheckerboardOverlay";

export function HandEyeTab() {
  // Preview
  const [preview, setPreview] = useState<HandEyePreview | null>(null);
  const [previewStale, setPreviewStale] = useState(false);

  // Captured poses
  const [poses, setPoses] = useState<PoseMeta[]>([]);

  // utils
  const [loading, setLoading] = useState(false);
  const motion = useMotion();
  const { torqueEnabled, enableTorque } = useJointControl();

  const handleCapture = async () => {
    console.log("Capturing pose...");
  };

  const handleReset = async () => {
    console.log("Resetting poses...");
  };

  const handleRemove = async (index: number) => {
    console.log("Removing pose at index:", index);
  };

  return (
    <div className="flex h-full gap-4">
      {/* 카메라 피드 */}
      <div className="flex-1">
        <CameraFeed
          className="h-2/3 w-full"
          overlay={
            <CheckerboardOverlay preview={preview} stale={previewStale} />
          }
        />
      </div>

      {/* 로봇 조작 */}
      <div className="w-56 shrink-0 flex flex-col gap-3">
        <div className="rounded-lg border bg-card p-4 flex flex-col gap-3">
          <h2 className="text-sm font-semibold">Move TCP</h2>

          <Button
            size="sm"
            variant={torqueEnabled ? "destructive" : "default"}
            onClick={() => enableTorque(!torqueEnabled)}
          >
            {torqueEnabled ? "Torque OFF" : "Torque ON"}
          </Button>
          <MoveTCPControl
            tcpPose={motion.tcpPose}
            loading={motion.loading}
            compact
            onMoveTCP={motion.moveTCP}
            onGetTCP={motion.getTCP}
          />
        </div>

        {motion.error && (
          <p className="text-xs text-destructive">{motion.error}</p>
        )}
      </div>

      {/* Workflow */}
      <div className="w-80 shrink-0 flex flex-col gap-3 overflow-y-auto">
        {/* Capture */}
        <div className="rounded-lg border bg-card p-4 flex flex-col gap-3">
          <h2 className="text-sm font-semibold">Hand-Eye — Capture</h2>
          <p className="text-xs text-muted-foreground">
            다양한 자세에서 캡처 (최소 3개, 권장 10+)
          </p>
          <div className="flex gap-2">
            <Button
              size="sm"
              className="flex-1"
              onClick={handleCapture}
              disabled={loading}
            >
              {loading ? "..." : "캡처"}
            </Button>
            <Button
              size="sm"
              variant="outline"
              onClick={handleReset}
              disabled={loading || poses.length === 0}
            >
              리셋
            </Button>
          </div>

          <HandEyePoseList
            poses={poses}
            onRemove={handleRemove}
            disabled={loading}
          />
        </div>
      </div>
    </div>
  );
}
