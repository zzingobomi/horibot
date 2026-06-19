/**
 * 직전 캘 자세 가져오기 — Phase 1 (수동 자유 자세) 에서 노출.
 *
 * 새 cal_run 시작 후 사용자가 *기존 commit 된 자세* 로 robot 이동만 시킬 수 있게.
 * 캘판이 그대로 고정돼 있으면 같은 자세 = 같은 board pose → capture-time fix
 * (timestamp align + stability wait + fixed exposure) 적용된 새 capture 로 σ 호전.
 *
 * flow:
 *   1. run dropdown — hand_eye kind, status=success runs 만
 *   2. 선택 → STORAGE_LIST_RUN_CAPTURES(run_id) → 자세 list 표시
 *   3. 각 row [이동] 버튼 → MOTION_MOVE_J 호출 (그 자세의 joint_angles, rad → deg)
 *   4. 사용자가 카메라 보고 직접 [캡처] (Phase 1 정상 흐름)
 */
import { useEffect, useState } from "react";
import { ChevronRight } from "lucide-react";
import { bridge } from "@/api/bridge";
import { ServiceKey } from "@/api/generated/contract";
import { PanelButton } from "@/components/shared/PanelButton";
import { Section } from "@/components/shared/Section";
import { useCalibrationRuns } from "@/hooks/useCalibrationRuns";
import type { components } from "@/api/generated/types";

type Capture = components["schemas"]["CalibrationCaptureRecord"];

const ARM_MOTOR_COUNT_FALLBACK = 6;

export function PoseImportSection({ robotId }: { robotId: string }) {
  const { runs } = useCalibrationRuns(robotId);
  const handEyeRuns = runs.filter((r) => r.run.kind === "hand_eye");

  // 첫 hand_eye run 자동 선택 — render-time derive (useEffect setState 회피).
  const [explicitRunId, setExplicitRunId] = useState<number | null>(null);
  const autoFirstId =
    explicitRunId === null && handEyeRuns.length > 0
      ? (handEyeRuns[0].run.id ?? null)
      : null;
  const selectedRunId = explicitRunId ?? autoFirstId;

  // captures: null = fetch 진행 중, [] = 빈 결과, [...] = 자세 list.
  const [captures, setCaptures] = useState<Capture[] | null>(null);
  const [fetchedRunId, setFetchedRunId] = useState<number | null>(null);
  const [moving, setMoving] = useState<number | null>(null);
  const [status, setStatus] = useState<string>("");

  // 선택된 run 의 captures fetch — selectedRunId 변하면 재호출.
  useEffect(() => {
    if (selectedRunId === null) return;
    let cancelled = false;
    (async () => {
      const res = await bridge.callService(
        ServiceKey.STORAGE_LIST_RUN_CAPTURES,
        { run_id: selectedRunId },
      );
      if (cancelled) return;
      setFetchedRunId(selectedRunId);
      if (res.success && res.data) {
        setCaptures(res.data.captures);
      } else {
        setCaptures([]);
        setStatus(`자세 목록 fetch 실패: ${res.message}`);
      }
    })();
    return () => {
      cancelled = true;
    };
  }, [selectedRunId]);

  const loadingCaptures =
    selectedRunId !== null && fetchedRunId !== selectedRunId;

  const handleMove = async (cap: Capture) => {
    setMoving(cap.pose_index);
    setStatus(`자세 #${cap.pose_index} 로 이동 중...`);
    // joint_angles (URDF rad) → JointDegree[]. motor id 는 1..N (arm motor id 순서).
    const n = cap.joint_angles.length || ARM_MOTOR_COUNT_FALLBACK;
    const joints = cap.joint_angles.slice(0, n).map((rad, i) => ({
      id: i + 1,
      degree: (rad * 180) / Math.PI,
    }));
    const res = await bridge.callService(
      ServiceKey.MOTION_MOVE_J,
      { joints },
      { robotId, timeoutMs: 30_000 },
    );
    setMoving(null);
    if (res.success) {
      setStatus(`자세 #${cap.pose_index} 도달 — 카메라 확인 후 [캡처] 누르세요.`);
    } else {
      setStatus(`이동 실패 #${cap.pose_index}: ${res.message}`);
    }
  };

  if (handEyeRuns.length === 0) return null;

  return (
    <Section label="직전 캘 자세 가져오기">
      <div className="flex flex-col gap-2">
        <p className="text-[11px] text-zinc-500 leading-snug font-mono">
          직전 캘 run 선택 → [이동] → 카메라 보고 [캡처]. 캘판 안 움직였으면
          그대로 사용 가능.
        </p>

        <select
          className="text-[11px] bg-zinc-900 border border-zinc-700 rounded px-2 py-1 text-zinc-200 font-mono"
          value={selectedRunId ?? ""}
          onChange={(e) =>
            setExplicitRunId(e.target.value ? Number(e.target.value) : null)
          }
        >
          <option value="">— run 선택 —</option>
          {handEyeRuns.map((r) => {
            const he = r.results.find((res) => res.kind === "hand_eye");
            const sigma =
              he && he.sigma_rot != null
                ? ` (σ=${he.sigma_rot.toFixed(2)}°)`
                : "";
            const ts = new Date((r.run.started_at ?? 0) * 1000)
              .toISOString()
              .slice(0, 16)
              .replace("T", " ");
            return (
              <option key={r.run.id ?? 0} value={r.run.id ?? 0}>
                run_id={r.run.id} {ts}
                {sigma}
              </option>
            );
          })}
        </select>

        {loadingCaptures && (
          <p className="text-[10px] text-zinc-500 font-mono">자세 fetch 중…</p>
        )}

        {!loadingCaptures && captures !== null && captures.length > 0 && (
          <div className="flex flex-col gap-1 max-h-[280px] overflow-y-auto pr-1">
            <span className="text-[10px] text-zinc-500 font-mono uppercase tracking-wide px-1">
              {captures.length}개 자세
            </span>
            {captures.map((cap) => (
              <div
                key={cap.pose_index}
                className="flex items-center justify-between gap-2 px-2 py-1 rounded border border-zinc-800/60 bg-zinc-900/30"
              >
                <span className="text-[11px] font-mono text-zinc-300 flex-shrink-0">
                  #{cap.pose_index}
                </span>
                <span className="text-[10px] font-mono text-zinc-500 flex-1 truncate">
                  [
                  {cap.joint_angles
                    .map((r) => ((r * 180) / Math.PI).toFixed(0))
                    .join(", ")}
                  ]°
                </span>
                <PanelButton
                  variant="outline"
                  className="!px-2 !py-0.5 !text-[10px] flex items-center gap-1"
                  onClick={() => void handleMove(cap)}
                  disabled={moving !== null}
                  title="이 자세로 robot 이동"
                >
                  <ChevronRight className="w-3 h-3" />
                  {moving === cap.pose_index ? "이동 중…" : "이동"}
                </PanelButton>
              </div>
            ))}
          </div>
        )}

        {status && (
          <p className="text-[10px] text-zinc-400 font-mono leading-snug px-1">
            {status}
          </p>
        )}
      </div>
    </Section>
  );
}
