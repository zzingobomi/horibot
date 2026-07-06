/**
 * CameraPanel — color 카메라 라이브 뷰 패널 (범용). scan mode 에서 자세 잡기 확인용
 * (오버레이 없는 순수 color 뷰). calibration 은 마커 오버레이가 필요해 별도
 * CalibrationCameraPanel 사용 — 범용 스트림을 calibration 에 결합하지 않기 위함.
 *
 * 패널 = router 의존(useParams) 자체 흡수 (registry 순수 유지, §4.1).
 */
import { useParams } from "react-router-dom";
import { CameraView } from "@/components/camera/CameraView";
import { DEFAULT_ROBOT_ID } from "@/constants";

export function CameraPanel() {
  const { id } = useParams<{ id: string }>();
  const robotId = id ?? DEFAULT_ROBOT_ID;
  return (
    <div className="h-full" data-testid="camera-panel">
      <CameraView robotId={robotId} />
    </div>
  );
}
