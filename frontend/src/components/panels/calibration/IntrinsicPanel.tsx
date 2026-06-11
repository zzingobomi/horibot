/**
 * Intrinsic calibration panel — D405 factory seed 가 있어 omx 시나리오에선 거의
 * 안 쓰임. USB UVC 카메라 swap 등 시에만 사용.
 *
 * 라이브 카메라는 [CalibrationCameraPanel] 이 공유 — 본 패널은 컨트롤 + 결과만.
 *
 * 진단 (Hand-Eye 와 같은 trauma-trap 회피 패턴):
 *   - RMS 색 분기 (good < 0.5px / warn < 1.0px / bad ≥ 1.0px)
 *   - RMS = 0.0 → "factory seed (재캘 안 함)" 회색 라벨 — 초록 trap 차단
 *   - 9 grid coverage 표시 — distortion model 이 frame 전 영역 generalize 됐는지
 *   - 캡처 실패 시 detect_full hint (마커 / 코너 부족 사유)
 */
import { useState } from "react";
import { Aperture } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { PanelShell } from "@/components/shared/PanelShell";
import { PanelButton } from "@/components/shared/PanelButton";
import { Section } from "@/components/shared/Section";
import { bridge } from "@/api/bridge";
import { ServiceKey } from "@/constants/topics";
import { useCalibrationStore } from "@/domain/stores/calibration";

type CaptureRes = {
  detected: boolean;
  captured_count: number;
  preview: string;
  hint?: string;
  coverage_count?: number;
};

type SaveRes = {
  rms_error: number;
  captured_count: number;
  coverage_count?: number;
  coverage_cells?: number[][]; // [[gx, gy], ...] — pydantic 직렬화는 number[][]
};

/**
 * RMS verdict — 색 분기. RMS=0 은 factory seed (재캘 안 함) 특수 케이스 분리.
 */
function rmsMeta(rms: number | null, good: number, warn: number): {
  color: string;
  label: string;
} {
  if (rms === null) return { color: "text-zinc-500", label: "— 미측정" };
  if (rms === 0)
    return {
      color: "text-zinc-500",
      label: "factory seed — 재캘 안 한 값",
    };
  if (rms < good) return { color: "text-green-400", label: "good (< " + good + "px)" };
  if (rms < warn) return { color: "text-amber-400", label: "warn (< " + warn + "px)" };
  return { color: "text-red-400", label: "bad (≥ " + warn + "px)" };
}

/**
 * 3×3 grid coverage 시각화 — 채운 cell 초록, 빈 cell 회색.
 */
function CoverageGrid({
  cells,
  recommended,
}: {
  cells: number[][];
  recommended: number;
}) {
  const set = new Set(cells.map(([gx, gy]) => `${gx},${gy}`));
  const count = set.size;
  return (
    <div className="flex flex-col gap-1">
      <div className="flex justify-between text-[10px] font-mono">
        <span className="text-zinc-500">Frame coverage</span>
        <span
          className={
            count >= recommended ? "text-green-400" : "text-amber-400"
          }
        >
          {count} / 9 cells
        </span>
      </div>
      <div className="grid grid-cols-3 gap-0.5 w-fit">
        {Array.from({ length: 9 }, (_, i) => {
          const gx = i % 3;
          const gy = Math.floor(i / 3);
          const filled = set.has(`${gx},${gy}`);
          return (
            <div
              key={i}
              className={`w-4 h-4 rounded-sm ${
                filled
                  ? "bg-green-500/70"
                  : "bg-zinc-800/80 border border-zinc-700/60"
              }`}
              title={`cell (${gx}, ${gy}) — ${filled ? "캡처됨" : "비어있음"}`}
            />
          );
        })}
      </div>
    </div>
  );
}

export function IntrinsicPanel(props: IDockviewPanelProps<object>) {
  const thresholds = useCalibrationStore((s) => s.thresholds);
  const rmsGood = thresholds?.intrinsic_rms_good_px ?? 0.5;
  const rmsWarn = thresholds?.intrinsic_rms_warn_px ?? 1.0;
  const minCaps = thresholds?.intrinsic_min_captures ?? 5;
  const recCaps = thresholds?.intrinsic_recommended_captures ?? 10;
  const coverageGood = thresholds?.intrinsic_grid_coverage_good ?? 7;

  const [captureCount, setCaptureCount] = useState(0);
  const [rmsError, setRmsError] = useState<number | null>(null);
  const [status, setStatus] = useState("");
  const [hint, setHint] = useState("");
  const [preview, setPreview] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [coverageCells, setCoverageCells] = useState<number[][]>([]);
  const [lastCoverageCount, setLastCoverageCount] = useState(0);

  const handleStart = async () => {
    await bridge.callService(ServiceKey.CALIB_INTRINSIC_START, {});
    setCaptureCount(0);
    setRmsError(null);
    setPreview(null);
    setHint("");
    setCoverageCells([]);
    setLastCoverageCount(0);
    setStatus("초기화됨. ChArUco 보드를 다양한 각도로 캡처하세요.");
  };

  const handleCapture = async () => {
    setLoading(true);
    const res = await bridge.callService(ServiceKey.CALIB_CAPTURE, {
      mode: "intrinsic",
    });
    setLoading(false);
    if (res.success) {
      const data = res.data as CaptureRes;
      setCaptureCount(data.captured_count);
      setLastCoverageCount(data.coverage_count ?? 0);
      setHint(data.hint ?? (data.detected ? "감지 성공" : "체커보드 미감지"));
      setStatus("");
      if (data.preview) setPreview(data.preview);
    } else {
      setHint(`실패: ${res.message}`);
    }
  };

  const handleSave = async () => {
    setLoading(true);
    const res = await bridge.callService(ServiceKey.CALIB_INTRINSIC_SAVE, {});
    setLoading(false);
    if (res.success) {
      const data = res.data as SaveRes;
      setRmsError(data.rms_error);
      setCoverageCells(data.coverage_cells ?? []);
      setStatus(`저장 완료 — RMS ${data.rms_error.toFixed(4)}px`);
    } else {
      setStatus(`실패: ${res.message}`);
    }
  };

  const rms = rmsMeta(rmsError, rmsGood, rmsWarn);

  return (
    <PanelShell
      icon={<Aperture className="w-3.5 h-3.5" />}
      title="Intrinsic"
      panelId={props.api.id}
      api={props.api}
      expandedHeight={520}
    >
      <Section label="안내">
        <p className="text-[11px] text-zinc-500 leading-snug font-mono">
          ChArUco 보드를 다양한 각도/거리/frame 위치에서 권장 {recCaps}장 (최소
          {" "}{minCaps}장) 캡처. omx + D405 시나리오에선 factory seed 사용 권장 — USB
          UVC 카메라일 때만 필요.
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
            disabled={captureCount < minCaps || loading}
          >
            {loading ? "..." : `캘리브레이션 & 저장 (≥ ${minCaps}장 필요)`}
          </PanelButton>
        </div>
      </Section>

      <Section label="Status">
        <div className="flex flex-col gap-2 font-mono text-[11px]">
          <div className="flex justify-between">
            <span className="text-zinc-500">캡처</span>
            <span
              className={
                captureCount >= recCaps
                  ? "text-green-400 tabular-nums"
                  : "text-amber-400 tabular-nums"
              }
            >
              {captureCount}장 (권장 {recCaps})
            </span>
          </div>
          <CoverageGrid
            cells={
              coverageCells.length > 0
                ? coverageCells
                : lastCoverageCount > 0
                  ? // 캡처 직후 count 만 응답 받았을 때 — cell 좌표 없으니 임시
                    // display: 채운 개수만 표시, 위치는 grid 비움.
                    []
                  : []
            }
            recommended={coverageGood}
          />
          {rmsError !== null && (
            <div className="flex flex-col gap-0.5">
              <div className="flex justify-between">
                <span className="text-zinc-500">RMS Error</span>
                <span className={`${rms.color} tabular-nums`}>
                  {rmsError.toFixed(4)}px
                </span>
              </div>
              <div className={`text-right text-[10px] ${rms.color}`}>
                {rms.label}
              </div>
            </div>
          )}
        </div>
        {hint && (
          <p className="text-[11px] text-amber-300 mt-2 leading-snug font-mono">
            {hint}
          </p>
        )}
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
