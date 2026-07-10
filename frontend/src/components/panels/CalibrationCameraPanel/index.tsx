/**
 * CalibrationCameraPanel — color 카메라 뷰 + ChArUco 마커 오버레이 + 캡처 안내 HUD
 * (calibrate mode). v1 카메라 뷰 패리티 — 자세 잡을 때 눈이 카메라에 가 있으므로
 * "카메라가 뭘 비추나(color) + 아루코 잡혔나(마커) + 지금 찍어도 되나(CaptureGuide)"
 * 를 한 화면에. 판정 텍스트/캡처 제어는 CalibrationPanel (같은 preview 스트림 소비).
 *
 * 범용 CameraView 위에 ChArUcoOverlay + CaptureGuide 를 children 으로 얹음. preview 는
 * CALIBRATION_PREVIEW(5Hz) — camera/stream(MJPEG) 과 별 채널이라 살짝 lag 하나 손
 * 자세 잡기엔 충분 (backend §CalibrationPreview).
 */
import { CameraView } from "@/components/camera/CameraView";
import { ChArUcoOverlay } from "@/components/camera/ChArUcoOverlay";
import { CaptureGuide } from "@/components/camera/CaptureGuide";
import { useRobotId } from "@/hooks/useRobotId";
import { useStream } from "@/framework";
import { Topic } from "@/api/generated/contract";
import type { CalibrationPreview } from "@/api/generated/contract";

export function CalibrationCameraPanel() {
  const robotId = useRobotId();

  const preview = useStream(Topic.CALIBRATION_PREVIEW, { robotId, staleMs: 1000 });
  const pv = (preview.value as CalibrationPreview | null) ?? null;

  return (
    <div className="h-full" data-testid="calibration-camera-panel">
      <CameraView robotId={robotId}>
        <ChArUcoOverlay preview={pv} />
        <CaptureGuide preview={pv} />
      </CameraView>
    </div>
  );
}
