import { BASE_URL, DEFAULT_ROBOT_ID } from "@/constants";

interface Props {
  className?: string;
  overlay?: React.ReactNode;
  robotId?: string;
}

export function CameraFeed({ className, overlay, robotId = DEFAULT_ROBOT_ID }: Props) {
  const cameraUrl = `${BASE_URL}/robots/${robotId}/camera/stream`;
  return (
    <div
      className={`relative overflow-hidden rounded-lg bg-black ${
        className ?? ""
      }`}
    >
      <img
        src={cameraUrl}
        alt="camera feed"
        className="w-full h-full object-contain"
      />
      {overlay && <div className="absolute inset-0">{overlay}</div>}
    </div>
  );
}
