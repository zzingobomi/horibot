import { useResource } from "@/framework";
import type { CalibrationResults } from "@/types/calibration";

/**
 * Hand-Eye/Intrinsic/joint_offsets 결과 — backend `/calibration/results` fetch.
 *
 * `useResource` module cache 가 cross-component sync — 다른 panel 도 동일 path
 * 호출하면 같은 응답 공유. COMMIT 후 호출자가 `refetch()` 호출 → 모든 사용처
 * 동시 갱신.
 */
export function useCalibrationResults() {
  const { data, loading, error, refetch } =
    useResource<CalibrationResults>("/calibration/results");
  return { results: data, loading, error, refetch };
}

/**
 * jointOffsets 만 motor_id → rad map 으로 derived. URDF 시각화 자리.
 *
 *   const offsets = useJointOffsetsRad();
 *   robot.setJointValue(name, base_rad + (offsets[id] ?? 0));
 */
export function useJointOffsetsRad(): Record<number, number> {
  const { data } = useResource<CalibrationResults, Record<number, number>>(
    "/calibration/results",
    {
      select: (d) =>
        Object.fromEntries(
          (d.joint_offsets ?? []).map((e) => [e.motor_id, e.offset_rad]),
        ),
    },
  );
  return data ?? {};
}
