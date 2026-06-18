/**
 * Parameter별 식별성 (Fisher CRLB) + staged gating 표시 — Phase 2.
 *
 * 각 보정 파라미터 block 이 "데이터로 얼마나 잘 결정됐나" 를 색 dot 으로.
 * 수치 노출 X — verdict (OK/WEAK/INSUFFICIENT) 만 (기존 observability 철학 유지).
 * INSUFFICIENT = 정보 부족으로 BA 가 추정 생략(freeze) — 자세 보강 신호.
 *
 * docs/handeye_ux_solver_v3_plan.md §3.5.
 */
import type {
  HandeyeParamObservabilityState,
  ParamVerdict,
} from "./types";

// block → 사용자 친화 라벨 + 한 줄 의미.
const BLOCK_META: { key: string; label: string; hint: string }[] = [
  { key: "handeye_rot", label: "카메라 회전", hint: "카메라↔EE 회전" },
  { key: "handeye_trans", label: "카메라 위치", hint: "카메라↔EE 이동" },
  { key: "joint_offset", label: "관절 영점", hint: "모터 zero 보정" },
  { key: "link", label: "링크 기하", hint: "URDF 링크 오차" },
  { key: "sag", label: "중력 처짐", hint: "자세 의존 sag" },
];

const VERDICT_DOT: Record<ParamVerdict, string> = {
  OK: "bg-emerald-400",
  WEAK: "bg-amber-400",
  INSUFFICIENT: "bg-zinc-600",
};

const VERDICT_TEXT: Record<ParamVerdict, string> = {
  OK: "잘 잡힘",
  WEAK: "보강 권장",
  INSUFFICIENT: "정보 부족",
};

export function ParamObservabilityCard({
  state,
}: {
  state: HandeyeParamObservabilityState | null;
}) {
  if (!state || !state.verdicts) return null;

  return (
    <div className="flex flex-col gap-1 rounded border border-zinc-800/60 bg-zinc-900/40 px-2 py-1.5">
      <span className="text-[10px] text-zinc-500 font-mono uppercase tracking-wide">
        보정값 식별성
      </span>
      <ul className="flex flex-col gap-0.5">
        {BLOCK_META.map(({ key, label, hint }) => {
          const v = state.verdicts[key];
          if (!v) return null;
          const frozen = v === "INSUFFICIENT";
          return (
            <li
              key={key}
              className="flex items-center gap-1.5 text-[11px] font-mono"
              title={hint}
            >
              <span
                className={`inline-block w-2 h-2 rounded-full shrink-0 ${VERDICT_DOT[v]}`}
              />
              <span className={frozen ? "text-zinc-500" : "text-zinc-300"}>
                {label}
              </span>
              <span className="ml-auto text-[10px] text-zinc-500">
                {VERDICT_TEXT[v]}
              </span>
            </li>
          );
        })}
      </ul>
    </div>
  );
}
