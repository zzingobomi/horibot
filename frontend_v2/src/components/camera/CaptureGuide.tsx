/**
 * CaptureGuide — 카메라 뷰 위 캡처 안내 HUD (v1 CaptureGuideOverlay 상당).
 *
 * 토크오프로 자세 잡을 때 눈이 카메라에 가 있으므로, "지금 찍어도 되나 / tilt" 판단을
 * 카메라 위에 직접 얹는다 (별 패널 흘끔거리지 않게). 판정(verdict)은 backend
 * capture_quality 가 tilt 임계(thresholds SSOT) 로 이미 계산 — 프론트는 재derive
 * 하지 않고 verdict 를 Badge variant 로 매핑만 (임계값 중복 X). tilt°/이유는 표시.
 *
 * verdict 색 = ui Badge 의 success/warning/destructive variant (traffic-light 한 세트).
 */
import { Badge } from "@/components/ui/badge";
import type { CalibrationPreview } from "@/api/generated/contract";

type BadgeVariant = React.ComponentProps<typeof Badge>["variant"];

const VERDICT_UI: Record<string, { variant: BadgeVariant; label: string }> = {
  green: { variant: "success", label: "지금 캡처 OK" },
  yellow: { variant: "warning", label: "자세 조정 권장" },
  red: { variant: "destructive", label: "캡처 불가" },
};

interface CaptureGuideProps {
  preview: CalibrationPreview | null;
}

export function CaptureGuide({ preview }: CaptureGuideProps) {
  if (!preview) return null;

  const ui = VERDICT_UI[preview.verdict] ?? {
    variant: "secondary" as const,
    label: preview.verdict,
  };
  const reason = preview.reasons?.[0];

  return (
    <div
      className="pointer-events-none absolute left-2 top-2 flex flex-col items-start gap-1 rounded bg-black/60 px-2 py-1.5"
      data-testid="capture-guide"
      data-verdict={preview.verdict}
    >
      <div className="flex items-center gap-1.5">
        <Badge variant={ui.variant}>{ui.label}</Badge>
        {preview.tilt_deg != null && (
          <span className="text-[11px] text-white/70" data-testid="capture-guide-tilt">
            tilt {preview.tilt_deg.toFixed(0)}°
          </span>
        )}
      </div>
      {reason && <span className="text-[11px] text-white/60">{reason}</span>}
    </div>
  );
}
