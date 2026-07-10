import { describe, it, expect } from "vitest";
import type { CalibrationBundle } from "@/api/generated/contract";
import { cameraInBase, frustumSegmentPositions } from "./cameraPose";

describe("cameraInBase", () => {
  it("hand_eye 없음(캘 전/mock) → identity fallback = TCP pose 그대로", () => {
    const pose = cameraInBase([0.2, 0.1, 0.3], [0, 0, 0, 1], null);
    expect(pose.position[0]).toBeCloseTo(0.2);
    expect(pose.position[1]).toBeCloseTo(0.1);
    expect(pose.position[2]).toBeCloseTo(0.3);
    expect(pose.quaternion[3]).toBeCloseTo(1);
  });

  it("hand_eye translation → TCP frame 에서 offset (R=I, t=[0,0,0.1])", () => {
    const bundle = {
      hand_eye: {
        result_data: {
          R_cam2gripper: [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
          ],
          t_cam2gripper: [[0], [0], [0.1]],
        },
      },
    } as unknown as CalibrationBundle;
    // TCP 회전 identity → 카메라 = TCP 위치 + z 0.1
    const pose = cameraInBase([0.2, 0, 0.3], [0, 0, 0, 1], bundle);
    expect(pose.position[2]).toBeCloseTo(0.4);
    // TCP 가 x축 180° 회전 ([1,0,0,0]) → offset 이 -z 로 뒤집힘
    const flipped = cameraInBase([0.2, 0, 0.3], [1, 0, 0, 0], bundle);
    expect(flipped.position[2]).toBeCloseTo(0.2);
  });
});

describe("frustumSegmentPositions", () => {
  it("8 선분 = 16 점 = 48 float, apex 는 원점, far rect 는 z=depth", () => {
    const depth = 0.12;
    const buf = frustumSegmentPositions(depth);
    expect(buf.length).toBe(48);
    // 앞 4 선분의 시작점 = apex(0,0,0)
    for (let seg = 0; seg < 4; seg++) {
      expect(buf[seg * 6 + 0]).toBe(0);
      expect(buf[seg * 6 + 1]).toBe(0);
      expect(buf[seg * 6 + 2]).toBe(0);
      expect(buf[seg * 6 + 5]).toBeCloseTo(depth); // 끝점 z = depth
    }
    // 뒤 4 선분(far rect)의 모든 점 z = depth
    for (let i = 24; i < 48; i += 3) {
      expect(buf[i + 2]).toBeCloseTo(depth);
    }
  });
});
