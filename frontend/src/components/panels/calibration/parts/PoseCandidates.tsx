import { useEffect, useState } from "react";
import { usePreviewStore } from "@/domain/stores/preview";
import type { NextPoseRecommendation, NoCandidatesReason } from "./types";

/** 추천 joints({id,degree}[]) → RobotModel jointAngles (rad, motor id 순서). */
function recToRad(rec: NextPoseRecommendation): number[] {
  return rec.joints.map((j) => (j.degree * Math.PI) / 180);
}

interface Props {
  // null: 아직 publish 안 됨 (Phase 2 진입 직후). []: 후보 0개 (noCandidatesReason).
  // [..]: 후보 N개.
  recommendations: NextPoseRecommendation[] | null;
  // 빈 추천 시 *어느 분기로* 떨어졌는지 — 분기별 명확 안내.
  noCandidatesReason?: NoCandidatesReason | null;
  /** ghost preview 대상 robot — 클릭한 후보 1개를 ghost 로 표시. */
  robotId: string;
}

/**
 * 빈 추천 분기별 메시지 + 색깔 — 모호 메시지 대신 분기별 한 가지로.
 */
function reasonMeta(reason: NoCandidatesReason | null | undefined): {
  text: string;
  color: string;
} {
  switch (reason) {
    case "sigma_sufficient_and_diverse":
      return {
        text: "캘 완료 — σ + 자세 다양성 모두 충족. [COMMIT] 권장.",
        color: "text-green-400",
      };
    case "sigma_sufficient_but_narrow":
      return {
        text: "σ 통과지만 자세 다양성 부족 — 위 식별성 카드의 약한 보정값 변주 자세로 추가 캡처 권장. [COMMIT] 도 가능.",
        color: "text-amber-400",
      };
    case "all_invisible":
      return {
        text: "추천 자세에서 보드가 카메라 시야 밖. 보드 위치 점검 또는 자유 자세로 [캡처].",
        color: "text-amber-400",
      };
    case "all_ik_fail":
      return {
        text: "추천 자세 IK 불가 — 보드가 로봇 reach 밖이거나 joint limits 충돌. 보드를 base 정면 + 적정 거리 (15-25cm) 로 옮겨주세요.",
        color: "text-amber-400",
      };
    case "insufficient_poses":
      return {
        text: "최소 캡처 수 미달 — 자유 자세로 더 캡처해주세요.",
        color: "text-zinc-400",
      };
    case "no_board_estimate":
      return {
        text: "보드 위치 추정 안 됨 — hand_eye / intrinsic 결과 없음. 캡처 더 진행.",
        color: "text-zinc-400",
      };
    default:
      return { text: "추천 후보 없음.", color: "text-zinc-500" };
  }
}

/**
 * 추천 후보 자세 리스트 — 클릭한 1개를 3D 씬에 주황 고스트로 표시.
 *
 * 사용자 흐름 (자동주행 X — 스펙: 시스템이 자세 대신 정하지 않음):
 *   1. 후보 N개 도착 → 목록에서 하나 클릭
 *   2. 그 후보가 주황 반투명 고스트로 3D 에 뜸 (동시에 여러 개 X — 헷갈림 방지)
 *   3. 토크오프로 직접 그 고스트 근처로 로봇 이동
 *   4. 카메라 overlay 🟢 (Traffic Light) 면 [캡처]
 */
export function PoseCandidates({
  recommendations,
  noCandidatesReason,
  robotId,
}: Props) {
  const [selected, setSelected] = useState<number | null>(null);
  const setGhost = usePreviewStore((s) => s.setGhost);

  // 후보 목록 바뀌면 selected 리셋 — render-time prev-value 패턴 (effect 내 setState 회피).
  const [prevRecs, setPrevRecs] = useState(recommendations);
  if (recommendations !== prevRecs) {
    setPrevRecs(recommendations);
    setSelected(null);
  }

  // ghost 정리 — 목록 바뀜 + unmount (setGhost 는 zustand store action). stale ghost 방지.
  useEffect(() => {
    setGhost(robotId, null);
    return () => setGhost(robotId, null);
  }, [recommendations, robotId, setGhost]);

  // 클릭 = 그 후보 1개 ghost 표시 (다시 클릭 = 해제). 화면 고정 → 토크오프 수동 매칭.
  const selectCandidate = (i: number, rec: NextPoseRecommendation) => {
    if (selected === i) {
      setSelected(null);
      setGhost(robotId, null);
    } else {
      setSelected(i);
      setGhost(robotId, recToRad(rec));
    }
  };

  if (recommendations === null) {
    return (
      <p className="text-[11px] text-zinc-500 leading-snug font-mono">
        추천 자세 계산 중...
      </p>
    );
  }

  if (recommendations.length === 0) {
    const meta = reasonMeta(noCandidatesReason);
    return (
      <p className={`text-[11px] leading-snug font-mono ${meta.color}`}>
        {meta.text}
      </p>
    );
  }

  return (
    <div className="flex flex-col gap-2">
      <span className="text-[10px] text-zinc-500 font-mono uppercase tracking-wide px-1">
        {recommendations.length}개 후보 · 클릭하면 고스트 표시
      </span>

      <ul className="flex flex-col gap-1">
        {recommendations.map((rec, i) => {
          const isSelected = selected === i;
          const visKnown = rec.visible !== undefined;
          const isInvisible = visKnown && rec.visible === false;
          return (
            <li
              key={i}
              className={
                "rounded border text-[11px] " +
                (isSelected
                  ? "border-orange-500/60 bg-orange-500/10"
                  : isInvisible
                    ? "border-zinc-800/40 bg-zinc-900/30 opacity-60"
                    : "border-zinc-800/60 bg-zinc-900/30")
              }
            >
              <button
                type="button"
                className="w-full flex items-center gap-1.5 px-2 py-1.5 text-left text-zinc-200 hover:text-zinc-100"
                onClick={() => selectCandidate(i, rec)}
                title={isSelected ? "고스트 끄기" : "이 후보 고스트로 보기"}
              >
                <span className="text-zinc-500 font-mono w-4 shrink-0 tabular-nums">
                  {i + 1}
                </span>
                <span className="font-mono truncate flex-1">{rec.label}</span>
                {isSelected && (
                  <span className="text-orange-400 shrink-0 text-[9px]" title="고스트 표시중">
                    ● 고스트
                  </span>
                )}
                {isInvisible && (
                  <span
                    className="text-amber-400 shrink-0 text-[9px] font-mono"
                    title={rec.visibility_reason ?? "보드 안 보임"}
                  >
                    ⚠ 안보임
                  </span>
                )}
              </button>

              {isSelected && (
                <div className="px-2 pb-2 pt-0 flex flex-col gap-1.5 border-t border-zinc-800/60">
                  <p className="text-[11px] text-zinc-400 leading-snug mt-1.5 font-mono">
                    {rec.reason}
                  </p>
                  <div className="rounded border border-zinc-800/60 bg-zinc-900/40 p-1.5 font-mono text-[10px] grid grid-cols-6 gap-x-2 gap-y-0.5">
                    {rec.joints.map((j, ji) => {
                      const isPrimary = ji === rec.primary_axis;
                      return (
                        <div
                          key={j.id}
                          className={
                            "flex flex-col items-center tabular-nums " +
                            (isPrimary ? "text-zinc-200 font-bold" : "text-zinc-500")
                          }
                        >
                          <span className="text-[9px]">J{ji + 1}</span>
                          <span>
                            {j.degree >= 0 ? "+" : ""}
                            {j.degree.toFixed(0)}°
                          </span>
                        </div>
                      );
                    })}
                  </div>
                </div>
              )}
            </li>
          );
        })}
      </ul>

      <p className="text-[10px] text-zinc-500 leading-snug px-1 font-mono">
        후보 클릭 → 고스트 표시 → 토크오프로 그 자세에 맞춤 → overlay 🟢 면 [캡처].
      </p>
    </div>
  );
}
