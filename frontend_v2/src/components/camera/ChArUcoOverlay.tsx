/**
 * ChArUcoOverlay — CalibrationPreview.corners_2d 를 CameraView color 이미지 위에 그림.
 *
 * SVG viewBox 를 원본 프레임 크기(image_width/height)로 두고 preserveAspectRatio 를
 * img 의 object-contain 과 동일(xMidYMid meet)하게 맞추면, 이미지가 스케일/letterbox
 * 돼도 코너 픽셀 좌표가 자동 정렬 — 수동 스케일 계산 불필요 (좌표계 SSOT = 원본 프레임).
 *
 * 색 = preview.verdict (green/yellow/red) — 자세 품질을 마커 색으로. 텍스트/판정은
 * CalibrationPanel 이 담당 (여기는 시각 오버레이만, 중복 X).
 */
import type { CalibrationPreview } from "@/api/generated/contract";

const VERDICT_STROKE: Record<string, string> = {
  green: "#22c55e",
  yellow: "#eab308",
  red: "#ef4444",
};

interface ChArUcoOverlayProps {
  preview: CalibrationPreview | null;
}

export function ChArUcoOverlay({ preview }: ChArUcoOverlayProps) {
  const w = preview?.image_width ?? 0;
  const h = preview?.image_height ?? 0;
  const corners = preview?.corners_2d ?? [];
  // 좌표계(원본 크기) 없거나 검출 코너 0 이면 그릴 것 없음.
  if (w <= 0 || h <= 0 || corners.length === 0) return null;

  const stroke = VERDICT_STROKE[preview?.verdict ?? "red"] ?? "#9ca3af";
  // 코너 반지름 — 원본 픽셀 기준 (viewBox 가 스케일 흡수). 프레임 대비 비례.
  const r = Math.max(3, Math.round(Math.min(w, h) * 0.008));

  return (
    <svg
      className="pointer-events-none absolute inset-0 h-full w-full"
      viewBox={`0 0 ${w} ${h}`}
      preserveAspectRatio="xMidYMid meet"
      data-testid="charuco-overlay"
      data-corner-count={corners.length}
    >
      {corners.map(([x, y], i) => (
        <circle
          key={i}
          cx={x}
          cy={y}
          r={r}
          fill="none"
          stroke={stroke}
          strokeWidth={r * 0.5}
        />
      ))}
    </svg>
  );
}
