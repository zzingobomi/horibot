import { BASE_URL } from "@/constants";
import { useCallback, useEffect, useState } from "react";
import { useRobotStore } from "@/store/robotStore";

export interface IntrinsicData {
  camera_matrix: number[][]; // 3x3
  dist_coeffs: number[][]; // 1xN
  image_size?: number[]; // [w, h]
}

export interface HandEyeData {
  R: number[][]; // 3x3 rotation matrix
  t: number[][]; // 3x1 translation [m]
  available_keys: string[];
}

export interface JointOffsetEntry {
  motor_id: number;
  offset_rad: number;
}

export interface CalibrationResults {
  intrinsic?: IntrinsicData;
  hand_eye?: HandEyeData;
  joint_offsets?: JointOffsetEntry[];
  intrinsic_error?: string;
  hand_eye_error?: string;
}

export interface CalibrationStatus {
  intrinsic: boolean;
  hand_eye: boolean;
}

/**
 * Hand-Eye/Intrinsic/joint_offsets 결과를 HTTP로 fetch.
 *
 * 분산 모드에서도 PC가 git 추적되는 같은 파일을 보므로 mount 시 한 번 fetch로
 * fresh 보장. COMMIT 후 호출자가 refetch() 호출하면 store에도 즉시 반영
 * (URDF가 백엔드와 동기). Zenoh 토픽 폐기됨.
 */
export function useCalibrationResults() {
  const [results, setResults] = useState<CalibrationResults | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const setJointOffsets = useRobotStore((s) => s.setJointOffsets);

  const fetchResults = useCallback(async () => {
    setLoading(true);
    setError(null);

    try {
      const res = await fetch(`${BASE_URL}/calibration/results`);
      if (!res.ok) {
        const err = await res.json().catch(() => null);
        throw new Error(err?.error || `HTTP ${res.status}`);
      }
      const data = (await res.json()) as CalibrationResults;
      setResults(data);

      // joint_offsets는 store로 푸시 → URDF가 백엔드와 즉시 동기.
      const map: Record<number, number> = {};
      for (const e of data.joint_offsets ?? []) map[e.motor_id] = e.offset_rad;
      setJointOffsets(map);
    } catch (e) {
      setResults(null);
      setError(e instanceof Error ? e.message : "Unknown error");
    } finally {
      setLoading(false);
    }
  }, [setJointOffsets]);

  useEffect(() => {
    fetchResults();
  }, [fetchResults]);

  return { results, loading, error, refetch: fetchResults };
}
