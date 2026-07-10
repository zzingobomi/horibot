/**
 * CalibrationScenePart — CalibrationPanel 의 scenePart (씬 기여 조각, 3D 표현).
 *
 * preview stream 의 board_in_cam(PnP board pose, camera frame)을 카메라 pose
 * (tcp corrected FK · hand_eye)와 합성해 ChArUco 보드를 로봇이 인식한 위치에
 * 렌더. 캘 전(hand_eye identity)엔 "검출이 살아있다"는 피드백, 캘 후엔 실물
 * 보드 위치와의 비교가 곧 hand_eye 품질의 육안 검증.
 *
 * scenePart 규약 ([docs/frontend.md]):
 *   - useRobotId() — 패널과 같은 멘탈모델 (RobotProvider 는 ScenePartHost 공급)
 *   - 패널 닫으면 보드도 사라짐 (인스턴스 lifecycle) + preview stream stale
 *     (2s) 시 숨김 — preview OFF 면 자연 소멸
 *   - 색 = DETECTION (인식 결과 — ground truth 아님을 색 체계가 전달)
 */
import { useMemo } from "react";
import * as THREE from "three";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type { CalibrationBundle } from "@/api/generated/contract";
import { useMirror, useStream } from "@/framework";
import { useRobotId } from "@/hooks/useRobotId";
import { cameraInBase } from "@/components/scene/objects/cameraPose";
import { RobotFrame } from "@/components/scene/shared/RobotFrame";
import { Frame } from "@/components/scene/shared/primitives";
import { VizColor } from "@/components/scene/theme/visualizationColors";

// ChArUco 보드 물리 크기 — backend board.py spec (7×5 squares × 25mm) 의
// 시각화용 사본 (SSOT 는 backend, 여기는 cosmetic 치수만).
const BOARD_W = 7 * 0.025;
const BOARD_H = 5 * 0.025;

export function CalibrationScenePart() {
  const robotId = useRobotId();
  const preview = useStream(Topic.CALIBRATION_PREVIEW, { robotId, staleMs: 2000 });
  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId });
  const bundle = useMirror({
    snapshotService: ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE,
    snapshotReq: { robot_id: robotId },
    changeTopic: Topic.CALIBRATION_ACTIVATED,
    robotId,
  });

  // board(base frame) = cam(base) · board_in_cam. base 변환은 <RobotFrame> 몫.
  const boardMatrix = useMemo(() => {
    const b = preview.value?.board_in_cam;
    if (!b || b.length !== 4 || !tcp.value) return null;
    const cam = cameraInBase(
      tcp.value.position,
      tcp.value.quaternion,
      (bundle.value ?? null) as CalibrationBundle | null,
    );
    const camM = new THREE.Matrix4().compose(
      new THREE.Vector3(cam.position[0], cam.position[1], cam.position[2]),
      new THREE.Quaternion(
        cam.quaternion[0],
        cam.quaternion[1],
        cam.quaternion[2],
        cam.quaternion[3],
      ),
      new THREE.Vector3(1, 1, 1),
    );
    // 4x4 row-major → Matrix4.set (row-major 인자)
    const boardM = new THREE.Matrix4().set(
      b[0][0], b[0][1], b[0][2], b[0][3],
      b[1][0], b[1][1], b[1][2], b[1][3],
      b[2][0], b[2][1], b[2][2], b[2][3],
      b[3][0], b[3][1], b[3][2], b[3][3],
    );
    return camM.multiply(boardM);
  }, [preview.value, tcp.value, bundle.value]);

  if (!boardMatrix || preview.stale) return null;

  return (
    <RobotFrame robotId={robotId}>
      <group matrix={boardMatrix} matrixAutoUpdate={false}>
        {/* 보드 평면 — ChArUco object point 원점이 코너라 중심 offset */}
        <mesh position={[BOARD_W / 2, BOARD_H / 2, 0]}>
          <planeGeometry args={[BOARD_W, BOARD_H]} />
          <meshBasicMaterial
            color={VizColor.DETECTION}
            transparent
            opacity={0.25}
            side={THREE.DoubleSide}
            depthWrite={false}
          />
        </mesh>
        <Frame
          pose={{ position: [0, 0, 0] }}
          size={0.03}
          label="board"
          labelColor={VizColor.DETECTION}
        />
      </group>
    </RobotFrame>
  );
}
