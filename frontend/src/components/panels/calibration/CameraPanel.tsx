/**
 * Calibration camera — Intrinsic / Hand-Eye 둘 다 공유하는 라이브 피드.
 *
 * overlay 는 ChArUco 검출 표시 (검출 박스 + tilt 알림) 만. Torque/Home/Jog 같은
 * robot 직접 제어는 RobotStatePanel 에 통합 — 카메라 위에 컨트롤 박지 않음.
 */
import { Camera } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { useParams } from "react-router-dom";
import { CameraFeed } from "@/components/shared/CameraFeed";
import { PanelShell } from "@/components/shared/PanelShell";
import { CaptureGuideOverlay } from "./parts/CaptureGuideOverlay";
import { useCalibrationStore } from "@/domain/stores/calibration";

export function CameraPanel(props: IDockviewPanelProps<object>) {
  const { id: robotId } = useParams<{ id: string }>();
  const preview = useCalibrationStore((s) => s.preview);
  const previewStale = useCalibrationStore((s) => s.previewStale);

  return (
    <PanelShell
      icon={<Camera className="w-3.5 h-3.5" />}
      title="Calibration Camera"
      panelId={props.api.id}
      api={props.api}
      expandedHeight={420}
    >
      <div
        className="relative w-full h-full bg-black overflow-hidden"
        style={{ minHeight: 200 }}
      >
        <CameraFeed
          className="!rounded-none w-full h-full"
          robotId={robotId}
          overlay={<CaptureGuideOverlay preview={preview} stale={previewStale} />}
        />
      </div>
    </PanelShell>
  );
}
