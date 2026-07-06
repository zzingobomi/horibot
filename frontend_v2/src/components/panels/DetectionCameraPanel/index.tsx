/**
 * DetectionCameraPanel — color 카메라 + 검출 bbox 오버레이 (v1 tasks 카메라 계승).
 *
 * backend detector 가 DETECT 마다 발행하는 DetectionsUpdate 스트림을 SVG 로 오버레이.
 * v2 detector 는 on-demand — task 의 search 자세에서 팔이 멈춰 검출하는 순간 bbox 가
 * 뜨고, 팔이 움직여 stale(useStream staleMs) 되면 자동 숨김 (그 시점 이미지 기준
 * 좌표라 팔 이동 후엔 무효). 최고 후보 = 초록, 나머지 = 회색.
 *
 * SVG viewBox = 검출 시점 이미지 크기, preserveAspectRatio 는 CameraView 의
 * object-contain 과 동일 정렬 (letterbox 여도 좌표 일치 — CameraView 설계 주석).
 */
import { useParams } from "react-router-dom";
import { CameraView } from "@/components/camera/CameraView";
import { DEFAULT_ROBOT_ID } from "@/constants";
import { useStream } from "@/framework";
import { Topic } from "@/api/generated/contract";

const STALE_MS = 8_000; // 검출 후 이 시간 지나면 오버레이 숨김 (팔 이동 대비)

export function DetectionCameraPanel() {
  const { id } = useParams<{ id: string }>();
  const robotId = id ?? DEFAULT_ROBOT_ID;

  const det = useStream(Topic.DETECTOR_DETECTIONS, { robotId, staleMs: STALE_MS });
  const v = det.value;
  const candidates = v?.candidates ?? [];
  const show = v != null && !det.stale && candidates.length > 0;

  return (
    <div className="h-full" data-testid="detection-camera-panel">
      <CameraView robotId={robotId}>
        {show && (
          <svg
            className="pointer-events-none absolute inset-0 h-full w-full"
            viewBox={`0 0 ${v.image_width} ${v.image_height}`}
            preserveAspectRatio="xMidYMid meet"
            data-testid="detection-overlay"
          >
            {candidates.map((c, i) => {
              const b = c.bbox_2d;
              if (!b) return null;
              const color = i === 0 ? "#34d399" : "#a1a1aa";
              return (
                <g key={i} data-testid="detection-bbox">
                  <rect
                    x={b[0]}
                    y={b[1]}
                    width={b[2] - b[0]}
                    height={b[3] - b[1]}
                    fill="none"
                    stroke={color}
                    strokeWidth={i === 0 ? 4 : 2}
                  />
                  <text
                    x={b[0]}
                    y={Math.max(b[1] - 6, 14)}
                    fill={color}
                    fontSize={18}
                    fontFamily="monospace"
                  >
                    {v.prompt} {(c.score * 100).toFixed(0)}%
                  </text>
                </g>
              );
            })}
          </svg>
        )}
      </CameraView>
    </div>
  );
}
