import { useEffect, useRef, useState } from "react";
import { Camera } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { PanelShell } from "@/components/canvas/ui/PanelShell";
import { CameraFeed } from "@/components/camera/CameraFeed";
import { useCameraStore } from "@/store/cameraStore";
import { useDetectorStore } from "@/store/detectorStore";

/**
 * Grounded detection bbox 오버레이 (이미지 px → 표시 px 스케일링).
 */
function GroundedBboxOverlay({
  frameWidth,
  frameHeight,
  displayWidth,
  displayHeight,
}: {
  frameWidth: number;
  frameHeight: number;
  displayWidth: number;
  displayHeight: number;
}) {
  const result = useDetectorStore((s) => s.groundedResult);
  if (!result || frameWidth <= 0 || frameHeight <= 0) return null;

  // object-contain 기준 letterbox 계산
  const frameRatio = frameWidth / frameHeight;
  const displayRatio = displayWidth / displayHeight;
  let renderW: number;
  let renderH: number;
  let offsetX = 0;
  let offsetY = 0;
  if (displayRatio > frameRatio) {
    renderH = displayHeight;
    renderW = renderH * frameRatio;
    offsetX = (displayWidth - renderW) / 2;
  } else {
    renderW = displayWidth;
    renderH = renderW / frameRatio;
    offsetY = (displayHeight - renderH) / 2;
  }

  const scaleX = renderW / frameWidth;
  const scaleY = renderH / frameHeight;
  const { x1, y1, x2, y2 } = result.bbox2d;
  const sx = x1 * scaleX + offsetX;
  const sy = y1 * scaleY + offsetY;
  const sw = (x2 - x1) * scaleX;
  const sh = (y2 - y1) * scaleY;

  return (
    <svg
      className="absolute inset-0 pointer-events-none"
      width={displayWidth}
      height={displayHeight}
    >
      <rect
        x={sx}
        y={sy}
        width={sw}
        height={sh}
        fill="none"
        stroke="#ff3366"
        strokeWidth={2}
      />
      <rect
        x={sx}
        y={Math.max(sy - 18, 0)}
        width={Math.max(40, sw * 0.55)}
        height={18}
        fill="rgba(255, 51, 102, 0.85)"
      />
      <text
        x={sx + 4}
        y={Math.max(sy - 18, 0) + 13}
        fontFamily="JetBrains Mono, monospace"
        fontSize={11}
        fontWeight={700}
        fill="white"
      >
        {result.prompt} {(result.confidence * 100).toFixed(0)}%
      </text>
    </svg>
  );
}

export function CameraFeedPanel(props: IDockviewPanelProps<object>) {
  const status = useCameraStore((s) => s.status);
  const containerRef = useRef<HTMLDivElement>(null);
  const [displaySize, setDisplaySize] = useState({ width: 0, height: 0 });

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const observer = new ResizeObserver((entries) => {
      const { width, height } = entries[0].contentRect;
      setDisplaySize({ width, height });
    });
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  const frameWidth = status?.width ?? 1280;
  const frameHeight = status?.height ?? 720;

  return (
    <PanelShell
      icon={<Camera className="w-3.5 h-3.5" />}
      title="Camera Feed"
      panelId={props.api.id}
      api={props.api}
    >
      <div
        ref={containerRef}
        className="relative w-full h-full bg-black overflow-hidden"
        style={{ minHeight: 160 }}
      >
        <CameraFeed className="!rounded-none w-full h-full" />
        {displaySize.width > 0 && (
          <GroundedBboxOverlay
            frameWidth={frameWidth}
            frameHeight={frameHeight}
            displayWidth={displaySize.width}
            displayHeight={displaySize.height}
          />
        )}
      </div>
    </PanelShell>
  );
}
