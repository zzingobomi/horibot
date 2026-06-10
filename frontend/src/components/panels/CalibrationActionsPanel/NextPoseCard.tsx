import { useState } from "react";
import { bridge } from "@/api/bridge";
import { Button } from "@/components/ui/button";
import { ServiceKey } from "@/constants/topics";
import type { NextPoseRecommendation } from "./types";

interface Props {
  // null: 아직 한 번도 publish 안 됨 (Phase 2 진입 직후 backend 보내기 전)
  // []  : 추천 후보 0개 (hand_eye / 보드 위치 추정 안 됨, 모든 anchor IK fail 등)
  // [..]: 후보 N개 (sphere shell anchor — 정면 / 좌 / 우 / 위 / 아래)
  recommendations: NextPoseRecommendation[] | null;
  visited: Set<number>;
  activeIndex: number | null;
  onMoved: (index: number) => void;
  // 사용자 명시 신호 — 추천 행의 [👎] 버튼 누름. anchor_id + 카테고리.
  // backend 가 fail mark + 다음 추천 제외.
  onReportFail?: (
    anchorId: string,
    category: "not_visible" | "red" | "motion_fail",
  ) => Promise<void> | void;
  disabled?: boolean;
}

/**
 * 다음 자세 후보 리스트.
 *
 * 사용자 흐름:
 *   1. [계산] 응답으로 N개 후보 도착
 *   2. 한 행의 [이동] 클릭 → move_j → 카메라 시선으로 체커보드 가시성 직접 확인
 *   3. 보이면 글로벌 [캡처](Capture 카드) 누름, 안 보이면 다른 행 [이동]
 *
 * 시각 마크:
 *   - visited ✓ : "이 행 [이동] 눌렀음" — 사용자 액션 그 자체. 추론 X.
 *   - active 파란 테두리 : "마지막으로 [이동]한 행" — 현재 선택의 시각 컨텍스트.
 *   캡처가 어느 행에서 됐는지는 *attribute하지 않음* — 사용자가 [이동] 후 수동
 *   조정해서 캡처할 수도 있어 거짓 양성 위험. 캡처 결과는 pose list로 확인.
 *
 * 리스트는 *다음 [계산] 전까지 고정*. 캡처해도 행 추가/삭제/재정렬 X.
 */
export function NextPoseCard({
  recommendations,
  visited,
  activeIndex,
  onMoved,
  onReportFail,
  disabled,
}: Props) {
  const [expanded, setExpanded] = useState<number | null>(null);
  const [movingIndex, setMovingIndex] = useState<number | null>(null);

  const handleMove = async (index: number, rec: NextPoseRecommendation) => {
    setMovingIndex(index);
    try {
      const res = await bridge.callService(
        ServiceKey.MOTION_MOVE_J,
        { joints: rec.joints },
        { timeoutMs: 30000 }
      );
      if (res.success) {
        onMoved(index);
      }
    } finally {
      setMovingIndex(null);
    }
  };

  // Phase 2 진입 직후 backend publish 전 (보통 짧음).
  if (recommendations === null) {
    return (
      <div className="flex flex-col gap-2">
        <p className="text-[11px] text-zinc-500 leading-snug">
          추천 자세 계산 중...
        </p>
      </div>
    );
  }

  // 추천 후보 0개 — 모든 anchor 가 IK fail / visibility fail / 사용자 명시 fail.
  if (recommendations.length === 0) {
    return (
      <div className="flex flex-col gap-2">
        <p className="text-[11px] text-zinc-500 leading-snug">
          추천 후보 없음 — 사용자 명시 fail 다수, IK 솔러블 자세 없음, 또는 σ
          충분히 낮음. [캡처] 자유 자세 시도 또는 [COMMIT].
        </p>
      </div>
    );
  }

  return (
    <div className="flex flex-col gap-2">
      <div className="flex items-center justify-between px-1">
        <span className="text-[10px] text-zinc-500 font-mono">
          {recommendations.length}개 후보
        </span>
      </div>

      <ul className="flex flex-col gap-1">
        {recommendations.map((rec, i) => {
          const isExpanded = expanded === i;
          const isMoving = movingIndex === i;
          const isVisited = visited.has(i);
          const isActive = activeIndex === i;
          // visibility gate — backend 가 보드 reproject 했을 때 화면 밖이면 false.
          // hard filter 아님 — 사용자가 [이동] 시도 가능하되 회색으로 hint.
          const visKnown = rec.visible !== undefined;
          const isInvisible = visKnown && rec.visible === false;
          return (
            <li
              key={i}
              className={
                "rounded-md border text-[11px] " +
                (isActive
                  ? "border-primary/60 bg-primary/5"
                  : isInvisible
                    ? "border-border/40 bg-muted/10 opacity-60"
                    : isVisited
                      ? "border-border bg-muted/30"
                      : "border-border bg-card")
              }
            >
              {/* 헤드라인 행 — 한 줄 압축 */}
              <div className="flex items-center gap-1.5 px-2 py-1.5">
                <span className="text-muted-foreground font-mono w-4 shrink-0">
                  {i + 1}
                </span>
                <button
                  type="button"
                  className="flex-1 flex items-center gap-1.5 min-w-0 text-left"
                  onClick={() => setExpanded(isExpanded ? null : i)}
                  title={isExpanded ? "접기" : "펼치기"}
                >
                  <span className="font-medium truncate">{rec.label}</span>
                  {isInvisible && (
                    <span
                      className="text-amber-500 shrink-0 text-[9px] font-mono"
                      title={rec.visibility_reason ?? "보드 안 보임"}
                    >
                      ⚠ 안보임
                    </span>
                  )}
                  {isVisited && (
                    <span
                      className="text-muted-foreground shrink-0"
                      title="이동함"
                    >
                      ✓
                    </span>
                  )}
                  <span className="text-muted-foreground shrink-0 text-[9px]">
                    {isExpanded ? "▾" : "▸"}
                  </span>
                </button>
                <Button
                  size="sm"
                  variant={isActive ? "default" : "outline"}
                  className="h-6 px-2 text-[10px] shrink-0"
                  onClick={() => handleMove(i, rec)}
                  disabled={disabled || movingIndex !== null}
                >
                  {isMoving ? "..." : "이동"}
                </Button>
              </div>

              {/* 펼침 — reason + joints + 명시 신호 [👎] 3종 */}
              {isExpanded && (
                <div className="px-2 pb-2 pt-0 flex flex-col gap-1.5 border-t border-border/50">
                  <p className="text-[10.5px] text-muted-foreground leading-snug mt-1.5">
                    {rec.reason}
                  </p>
                  {onReportFail && rec.diagnostics?.anchor_id && (
                    <div className="flex flex-col gap-1">
                      <p className="text-[9.5px] text-zinc-500">
                        이 자세 별로 — 사유 알려줘 (backend 가 다음 추천 제외):
                      </p>
                      <div className="flex gap-1 flex-wrap">
                        <Button
                          size="sm"
                          variant="outline"
                          className="h-5 px-1.5 text-[9.5px]"
                          onClick={() =>
                            void onReportFail(
                              String(rec.diagnostics?.anchor_id ?? ""),
                              "not_visible",
                            )
                          }
                          disabled={disabled}
                          title="도달 후 보드 화면 밖"
                        >
                          👎 안 보임
                        </Button>
                        <Button
                          size="sm"
                          variant="outline"
                          className="h-5 px-1.5 text-[9.5px]"
                          onClick={() =>
                            void onReportFail(
                              String(rec.diagnostics?.anchor_id ?? ""),
                              "red",
                            )
                          }
                          disabled={disabled}
                          title="보이는데 overlay 빨강 (tilt extreme / 코너 부족)"
                        >
                          👎 빨강
                        </Button>
                        <Button
                          size="sm"
                          variant="outline"
                          className="h-5 px-1.5 text-[9.5px]"
                          onClick={() =>
                            void onReportFail(
                              String(rec.diagnostics?.anchor_id ?? ""),
                              "motion_fail",
                            )
                          }
                          disabled={disabled}
                          title="도달 실패 (motion 자체 fail)"
                        >
                          👎 도달 실패
                        </Button>
                      </div>
                    </div>
                  )}
                  <div className="rounded bg-muted/40 p-1.5 font-mono text-[10px] grid grid-cols-5 gap-x-2 gap-y-0.5">
                    {rec.joints.map((j, ji) => {
                      const isPrimary = ji === rec.primary_axis;
                      return (
                        <div
                          key={j.id}
                          className={
                            "flex flex-col items-center " +
                            (isPrimary
                              ? "text-foreground font-semibold"
                              : "text-muted-foreground")
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

      <p className="text-[10px] text-muted-foreground leading-snug px-1">
        [이동] 후 카메라에서 체커보드 보이면 아래 [캡처]를 누르세요. 안 보이면
        다음 행 [이동].
      </p>
    </div>
  );
}
