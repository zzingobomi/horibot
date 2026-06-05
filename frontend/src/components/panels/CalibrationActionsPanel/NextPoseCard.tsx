import { useState } from "react";
import { bridge } from "@/api/bridge";
import { Button } from "@/components/ui/button";
import { ServiceKey } from "@/constants/topics";
import type { NextPoseRecommendation } from "./types";

interface Props {
  // null: 아직 한 번도 계산 안 됨 (초기 안내 표시)
  // []  : 계산했지만 추천 후보 0개 (σ 충분히 좋거나 변주 여유 없음)
  // [..]: 후보 N개
  recommendations: NextPoseRecommendation[] | null;
  visited: Set<number>;
  activeIndex: number | null;
  onMoved: (index: number) => void;
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

  // 초기 안내 (계산 전)
  if (recommendations === null) {
    return (
      <div className="rounded-lg border bg-card p-4 flex flex-col gap-2">
        <h2 className="text-sm font-semibold">다음 자세 추천</h2>
        <p className="text-xs text-muted-foreground leading-snug">
          직접 자세 잡고 [캡처]를 몇 번 (권장 10장) 누른 뒤 [계산]을 한 번
          누르세요. 그러면 다음 자세 후보 리스트가 여기 표시됩니다. 이후
          [이동]→카메라 확인→[캡처]→[계산] 반복.
        </p>
      </div>
    );
  }

  // 계산했지만 후보 0개
  if (recommendations.length === 0) {
    return (
      <div className="rounded-lg border bg-card p-4 flex flex-col gap-2">
        <h2 className="text-sm font-semibold">다음 자세 추천</h2>
        <p className="text-xs text-muted-foreground leading-snug">
          추천 후보 없음 — σ가 충분히 낮거나 변주 여유 없음. 결과 확인 후 만족
          하면 COMMIT, 더 줄이고 싶으면 자유 캡처 후 [계산].
        </p>
      </div>
    );
  }

  return (
    <div className="rounded-lg border bg-card p-3 flex flex-col gap-2 shrink-0">
      <div className="flex items-center justify-between px-1">
        <h2 className="text-sm font-semibold">다음 자세 추천</h2>
        <span className="text-[10px] text-muted-foreground font-mono">
          {recommendations.length}개 후보
        </span>
      </div>

      <ul className="flex flex-col gap-1">
        {recommendations.map((rec, i) => {
          const isExpanded = expanded === i;
          const isMoving = movingIndex === i;
          const isVisited = visited.has(i);
          const isActive = activeIndex === i;
          return (
            <li
              key={i}
              className={
                "rounded-md border text-[11px] " +
                (isActive
                  ? "border-primary/60 bg-primary/5"
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

              {/* 펼침 — reason 텍스트 + joint 5개 */}
              {isExpanded && (
                <div className="px-2 pb-2 pt-0 flex flex-col gap-1.5 border-t border-border/50">
                  <p className="text-[10.5px] text-muted-foreground leading-snug mt-1.5">
                    {rec.reason}
                  </p>
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
