/**
 * CameraView — robot color 카메라 라이브 뷰 (범용, 재사용).
 *
 * backend bridge `/robots/{id}/camera/stream` (color MJPEG, multipart) 를 <img> 로
 * 렌더. calibration(ChArUco 오버레이) / scan(자세 확인) 양쪽이 공유 — depth(rgbd)
 * 무관, has_camera(camera_backend 유무) 로 게이팅. 오버레이는 children 슬롯으로 얹음
 * (관심사 분리 — 범용 뷰는 color 만, calibration 마커는 CalibrationCameraPanel).
 *
 * 스트림 URL SSOT = BASE_URL (constants). img 는 object-contain — 오버레이 SVG 도
 * 같은 preserveAspectRatio 로 정렬하면 letterbox 여도 좌표가 맞는다.
 */
import { useState } from "react";
import { BASE_URL } from "@/constants";
import { useRobots } from "@/hooks/useRobots";

interface CameraViewProps {
  robotId: string;
  /** color 이미지 위에 절대배치로 얹을 오버레이 (예: ChArUco 마커 SVG). */
  children?: React.ReactNode;
}

export function CameraView({ robotId, children }: CameraViewProps) {
  const { robots } = useRobots();
  const [broken, setBroken] = useState(false);

  const robot = robots.find((r) => r.id === robotId);
  const hasCamera = robot?.has_camera ?? false;

  if (!hasCamera) {
    return (
      <div
        className="flex h-full items-center justify-center text-[12px] text-muted-foreground"
        data-testid="camera-view"
        data-has-camera="false"
      >
        이 robot 은 카메라가 없습니다
      </div>
    );
  }

  const url = `${BASE_URL}/robots/${robotId}/camera/stream`;

  return (
    <div
      className="relative h-full w-full bg-black"
      data-testid="camera-view"
      data-has-camera="true"
    >
      <img
        src={url}
        alt={`${robotId} camera`}
        className="absolute inset-0 h-full w-full object-contain"
        data-testid="camera-stream"
        onError={() => setBroken(true)}
        onLoad={() => setBroken(false)}
      />
      {children}
      {broken && (
        <div
          className="absolute inset-x-0 top-0 bg-red-950/70 px-2 py-1 text-[12px] text-red-200"
          data-testid="camera-broken"
        >
          카메라 스트림 연결 안 됨 — backend / 카메라 노드 확인
        </div>
      )}
    </div>
  );
}
