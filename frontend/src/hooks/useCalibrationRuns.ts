/**
 * Run 단위 캘 history — `STORAGE_LIST_CALIBRATION_RUNS` 호출 + invalidation 자동 refetch.
 *
 * storage_layer.md §13.7 Stage 4 design A:
 *   - 한 row = 한 Run (= 한 캘 세션). 펼치면 5 kind result 펼침.
 *   - ACTIVATE 후 backend 가 `STORAGE_CALIBRATION_INVALIDATED` publish → 본 hook
 *     이 받아 자동 refetch (race window 없음).
 *
 * 사용:
 *   const { runs, loading, refetch, activate } = useCalibrationRuns(robotId);
 */
import { useCallback, useEffect, useState } from "react";
import { bridge } from "@/api/bridge";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type { components } from "@/api/generated/types";

export type CalibrationRunSummary = components["schemas"]["CalibrationRunSummary"];
export type CalibrationResultRecord =
  components["schemas"]["ListCalibrationRunsRes"]["runs"][number]["results"][number];

const LIMIT = 50;

export function useCalibrationRuns(robotId: string) {
  const [runs, setRuns] = useState<CalibrationRunSummary[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // mount + robotId 변경 + invalidation 시 fetch. cancellation flag 로 stale
  // setState 차단 (unmount / robot 전환 race).
  useEffect(() => {
    let cancelled = false;
    const load = () => {
      setLoading(true);
      setError(null);
      bridge
        .callService(ServiceKey.STORAGE_LIST_CALIBRATION_RUNS, {
          robot_id: robotId,
          limit: LIMIT,
        })
        .then((res) => {
          if (cancelled) return;
          setLoading(false);
          if (res.success && res.data) {
            setRuns(res.data.runs);
          } else {
            setError(res.message || "list_runs 실패");
            setRuns([]);
          }
        });
    };
    load();
    const unsub = bridge.subscribe(
      Topic.STORAGE_CALIBRATION_INVALIDATED,
      (msg) => {
        if (cancelled) return;
        if (msg.robot_id === robotId) load();
      },
    );
    return () => {
      cancelled = true;
      unsub();
    };
  }, [robotId]);

  // 사용자 명시 "새로고침" 버튼 — effect 의 load 와 동일 자리 호출.
  const refetch = useCallback(() => {
    setLoading(true);
    setError(null);
    bridge
      .callService(ServiceKey.STORAGE_LIST_CALIBRATION_RUNS, {
        robot_id: robotId,
        limit: LIMIT,
      })
      .then((res) => {
        setLoading(false);
        if (res.success && res.data) {
          setRuns(res.data.runs);
        } else {
          setError(res.message || "list_runs 실패");
          setRuns([]);
        }
      });
  }, [robotId]);

  // ACTIVATE — caller (panel) 가 result_id 받아 호출. 성공 시 invalidation 으로
  // 자동 refetch 되므로 여기서 직접 setRuns 안 함 (single source of truth).
  const activate = useCallback(
    async (resultId: number): Promise<{ success: boolean; message: string }> => {
      const res = await bridge.callService(
        ServiceKey.STORAGE_ACTIVATE_CALIBRATION,
        { result_id: resultId },
      );
      return { success: res.success, message: res.message };
    },
    [],
  );

  return { runs, loading, error, refetch, activate };
}
