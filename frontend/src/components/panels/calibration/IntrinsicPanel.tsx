/**
 * Intrinsic calibration panel — D405 factory seed 가 있어 omx 시나리오에선 거의
 * 안 쓰임. USB UVC 카메라 swap 등 시에만 사용.
 *
 * 라이브 카메라는 [CalibrationCameraPanel] 이 공유 — 본 패널은 컨트롤 + 결과만.
 */
import { useState } from "react";
import { Aperture } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { PanelShell } from "@/components/shared/PanelShell";
import { PanelButton } from "@/components/shared/PanelButton";
import { Section } from "@/components/shared/Section";
import { bridge } from "@/api/bridge";
import { ServiceKey } from "@/constants/topics";

export function IntrinsicPanel(props: IDockviewPanelProps<object>) {
  const [captureCount, setCaptureCount] = useState(0);
  const [rmsError, setRmsError] = useState<number | null>(null);
  const [status, setStatus] = useState("");
  const [preview, setPreview] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const handleStart = async () => {
    await bridge.callService(ServiceKey.CALIB_INTRINSIC_START, {});
    setCaptureCount(0);
    setRmsError(null);
    setPreview(null);
    setStatus("초기화됨. ChArUco 보드를 다양한 각도로 캡처하세요.");
  };

  const handleCapture = async () => {
    setLoading(true);
    const res = await bridge.callService(ServiceKey.CALIB_CAPTURE, {
      mode: "intrinsic",
    });
    setLoading(false);
    if (res.success) {
      const data = res.data as {
        detected: boolean;
        captured_count: number;
        preview: string;
      };
      setCaptureCount(data.captured_count);
      setStatus(
        data.detected
          ? `감지 성공 (${data.captured_count}장)`
          : "체커보드 미감지",
      );
      if (data.preview) setPreview(data.preview);
    }
  };

  const handleSave = async () => {
    setLoading(true);
    const res = await bridge.callService(ServiceKey.CALIB_INTRINSIC_SAVE, {});
    setLoading(false);
    if (res.success) {
      const data = res.data as { rms_error: number };
      setRmsError(data.rms_error);
      setStatus(`저장 완료 (RMS: ${data.rms_error.toFixed(4)})`);
    } else {
      setStatus(`실패: ${res.message}`);
    }
  };

  return (
    <PanelShell
      icon={<Aperture className="w-3.5 h-3.5" />}
      title="Intrinsic"
      panelId={props.api.id}
      api={props.api}
      expandedHeight={420}
    >
      <Section label="안내">
        <p className="text-[11px] text-zinc-500 leading-snug font-mono">
          ChArUco 보드를 다양한 각도/거리에서 최소 10장 캡처. omx + D405
          시나리오에선 factory seed 사용 권장 — USB UVC 카메라일 때만 필요.
        </p>
      </Section>

      <Section label="Capture">
        <div className="flex flex-col gap-2">
          <div className="flex gap-2">
            <PanelButton
              variant="outline"
              onClick={() => void handleStart()}
              disabled={loading}
              className="flex-1"
            >
              초기화
            </PanelButton>
            <PanelButton
              variant="primary"
              onClick={() => void handleCapture()}
              disabled={loading}
              className="flex-1"
            >
              {loading ? "..." : "캡처"}
            </PanelButton>
          </div>
          <PanelButton
            variant="secondary"
            onClick={() => void handleSave()}
            disabled={captureCount < 5 || loading}
          >
            {loading ? "..." : "캘리브레이션 & 저장"}
          </PanelButton>
        </div>
      </Section>

      <Section label="Status">
        <div className="flex flex-col gap-1 font-mono text-[11px]">
          <div className="flex justify-between">
            <span className="text-zinc-500">캡처</span>
            <span className="text-zinc-200 tabular-nums">
              {captureCount}장
            </span>
          </div>
          {rmsError !== null && (
            <div className="flex justify-between">
              <span className="text-zinc-500">RMS Error</span>
              <span className="text-emerald-400 tabular-nums">
                {rmsError.toFixed(4)}
              </span>
            </div>
          )}
        </div>
        {status && (
          <p className="text-[11px] text-zinc-400 mt-2 leading-snug font-mono">
            {status}
          </p>
        )}
      </Section>

      {preview && (
        <Section label="Last Capture">
          <img
            src={`data:image/jpeg;base64,${preview}`}
            className="w-full rounded object-contain border border-zinc-800/60"
            alt="preview"
          />
        </Section>
      )}
    </PanelShell>
  );
}
