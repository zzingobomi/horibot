/**
 * DetectionCameraPanel — color 카메라 + 검출 오버레이 (v1 tasks 카메라 계승).
 *
 * 두 스트림을 소비: DETECTIONS(bbox) / DETECTIONS_ORIENTED(bbox + obb + mask contour,
 * [DRAFT] 회전 파지). oriented 가 fresh 면 그걸 우선 (obb 회전 사각형 + SAM mask 실루엣
 * + grasp yaw 라벨), 없으면 plain bbox. 둘 다 on-demand — task search 자세에서 검출하는
 * 순간 뜨고, 팔이 움직여 stale 되면 자동 숨김 (그 시점 이미지 기준 좌표라 무효).
 *
 * SVG viewBox = 검출 시점 이미지 크기, preserveAspectRatio 는 CameraView 의
 * object-contain 과 동일 정렬 (letterbox 여도 좌표 일치 — CameraView 설계 주석).
 *
 * 카메라 피드는 물리적으로 robot 하나에 묶이므로 task 패널이 아니라 robot-scoped
 * 패널 — robot 은 robotOwnership(capability=rgbd, registry)이 공급 (task 바인딩
 * 서비스와 무관).
 */
import { CameraView } from "@/components/camera/CameraView";
import { useStream } from "@/framework";
import { useRobotId } from "@/hooks/useRobotId";
import { Topic } from "@/api/generated/contract";

const STALE_MS = 8_000; // 검출 후 이 시간 지나면 오버레이 숨김 (팔 이동 대비)

const BEST = "#34d399"; // 최고 후보 bbox (초록)
const REST = "#a1a1aa"; // 나머지 bbox (회색)
const OBB = "#f59e0b"; // 회전 파지 사각형 (호박)
// SAM mask 실루엣 — 초록(bbox)과 확실히 다른 색 (옛 하늘색 #38bdf8 은 작은 화면에서
// 초록과 혼동 — 2026-07-09 "AABB 가 회전됐다" 오독). 반투명 채움 = 세그멘테이션
// 영역 실시간 확인 (best 후보만 채움 — 겹침 후보 알파 중첩 방지).
const CONTOUR = "#d946ef"; // fuchsia

function points(pts: [number, number][]): string {
  return pts.map(([x, y]) => `${x},${y}`).join(" ");
}

// 오버레이가 읽는 필드 (plain Detection = obb/contour/yaw 부재). DETECTIONS(bbox 만) /
// DETECTIONS_ORIENTED(전부) 두 스트림 candidate 를 하나로 취급.
type OverlayCand = {
  score: number;
  bbox_2d?: [number, number, number, number] | null;
  obb_2d?: [number, number][] | null;
  mask_contour?: [number, number][] | null;
  grasp_yaw?: number | null;
};

export function DetectionCameraPanel() {
  const robotId = useRobotId();

  const det = useStream(Topic.DETECTOR_DETECTIONS, { robotId, staleMs: STALE_MS });
  const ori = useStream(Topic.DETECTOR_DETECTIONS_ORIENTED, {
    robotId,
    staleMs: STALE_MS,
  });

  // oriented 우선 (obb + contour 有). 둘 다 없거나 stale 이면 숨김.
  const oriented = ori.value != null && !ori.stale ? ori.value : null;
  const plain = det.value != null && !det.stale ? det.value : null;
  const src = oriented ?? plain;
  const candidates = (src?.candidates ?? []) as OverlayCand[];
  const show = src != null && candidates.length > 0;

  return (
    <div className="h-full" data-testid="detection-camera-panel">
      <CameraView robotId={robotId}>
        {show && (
          <svg
            className="pointer-events-none absolute inset-0 h-full w-full"
            viewBox={`0 0 ${src.image_width} ${src.image_height}`}
            preserveAspectRatio="xMidYMid meet"
            data-testid="detection-overlay"
          >
            {candidates.map((c, i) => {
              const b = c.bbox_2d;
              const color = i === 0 ? BEST : REST;
              // oriented 후보만 obb/contour/yaw 보유 (DRAFT — 회전 파지). plain=undefined.
              const obb = c.obb_2d;
              const contour = c.mask_contour;
              const yaw = c.grasp_yaw;
              return (
                <g key={i} data-testid="detection-bbox">
                  {contour && (
                    <polygon
                      data-testid="detection-contour"
                      points={points(contour)}
                      fill={i === 0 ? CONTOUR : "none"}
                      fillOpacity={0.3}
                      stroke={CONTOUR}
                      strokeWidth={2}
                      strokeOpacity={0.9}
                    />
                  )}
                  {obb && (
                    <polygon
                      data-testid="detection-obb"
                      points={points(obb)}
                      fill="none"
                      stroke={OBB}
                      strokeWidth={3}
                    />
                  )}
                  {b && (
                    <rect
                      x={b[0]}
                      y={b[1]}
                      width={b[2] - b[0]}
                      height={b[3] - b[1]}
                      fill="none"
                      stroke={color}
                      strokeWidth={i === 0 ? 4 : 2}
                    />
                  )}
                  {/* 라벨은 best 후보만 — top_k 후보가 같은 물체에 겹치면 같은 자리
                      텍스트가 뭉개져 깨짐 (2026-07-09 실물 확인). */}
                  {b && i === 0 && (
                    <text
                      x={b[0]}
                      y={Math.max(b[1] - 6, 14)}
                      fill={color}
                      fontSize={18}
                      fontFamily="monospace"
                    >
                      {src.prompt} {(c.score * 100).toFixed(0)}%
                      {yaw != null ? ` ∠${((yaw * 180) / Math.PI).toFixed(0)}°` : ""}
                    </text>
                  )}
                </g>
              );
            })}
          </svg>
        )}
      </CameraView>
    </div>
  );
}
