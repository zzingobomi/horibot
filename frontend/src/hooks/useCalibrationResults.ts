import { BASE_URL } from "@/constants";
import { useCallback, useEffect, useState } from "react";

export interface IntrinsicData {
  camera_matrix: number[][]; // 3x3
  dist_coeffs: number[][]; // 1xN
  image_size?: number[]; // [w, h]
}

export interface HandEyeData {
  R: number[][]; // 3x3 rotation matrix (camera → base or EEF → camera)
  t: number[][]; // 3x1 translation [m]
  available_keys: string[];
}

export interface CalibrationResults {
  intrinsic?: IntrinsicData;
  hand_eye?: HandEyeData;
  joint_offsets_rad?: number[]; // BA 결과. 길이 5 (joint 1..5).
  intrinsic_error?: string;
  hand_eye_error?: string;
}

export interface CalibrationStatus {
  intrinsic: boolean;
  hand_eye: boolean;
}

export function useCalibrationResults() {
  const [results, setResults] = useState<CalibrationResults | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const fetchResults = useCallback(async () => {
    setLoading(true);
    setError(null);

    try {
      const res = await fetch(`${BASE_URL}/calibration/results`);

      if (!res.ok) {
        const err = await res.json().catch(() => null);
        throw new Error(err?.error || `HTTP ${res.status}`);
      }

      const data = await res.json();
      setResults(data);
    } catch (e) {
      setResults(null);
      setError(e instanceof Error ? e.message : "Unknown error");
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchResults();
  }, [fetchResults]);

  return { results, loading, error, refetch: fetchResults };
}
