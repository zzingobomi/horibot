import { useCallback, useState } from "react";
import { Bot, Play, Square } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { PanelShell } from "@/components/canvas/ui/PanelShell";
import { Section } from "@/components/canvas/ui/Section";
import { useTask } from "@/hooks/useTask";
import { useSelfPlayStore } from "@/store/selfPlayStore";
import type { SelfPlayStage, StageResult } from "@/types/self_play";

// 객체 type → prompt + gripper preset. backend (core/gripper_setup.py 의
// GripperSetup) 가 받는 raw 값. 종이컵 → 박스 → 큐브 순으로 ramp-up
// (docs/self_play_pick.md 결정 로그 #6).
//
// gripper raw position semantics:
//   - 작을수록 닫힘 (close default = 1800), 클수록 열림 (open default = 2600).
//   - held_threshold: close 후 position 이 이 값 ↑ 이면 잡힘, ↓ 이면 빈손.
//     큰 객체일수록 close position 이 더 큰 값에서 멈추므로 threshold 도 ↑.
//
// ⚠ 1회 calib 권장 — 첫 단계 진입 시 실측 후 값 조정.

type ObjectType = "paper_cup" | "box" | "cube";

interface GripperPreset {
  close_current?: number;
  open_position?: number;
  close_position?: number;
  held_threshold?: number;
}

const OBJECT_PRESETS: Record<
  ObjectType,
  { label: string; prompt: string; gripper: GripperPreset }
> = {
  paper_cup: {
    label: "Paper Cup (~80mm)",
    prompt: "white paper cup",
    gripper: {
      close_current: 100, // 찌그러짐 방지 (default 200 보다 약함)
      held_threshold: 2000, // 폭 큼 → close 가 더 큰 raw 에서 멈춤
    },
  },
  box: {
    label: "Box (~50mm)",
    prompt: "brown paper box",
    gripper: {
      close_current: 200,
      held_threshold: 1950,
    },
  },
  cube: {
    label: "Cube (~20mm)",
    prompt: "white calibration cube",
    gripper: {
      close_current: 200,
      held_threshold: 1900,
    },
  },
};

const DEFAULT_OBJECT: ObjectType = "paper_cup";

const STAGE_COLOR: Record<SelfPlayStage, string> = {
  idle: "text-zinc-500",
  starting: "text-sky-400",
  detecting: "text-sky-400",
  hovering: "text-cyan-400",
  descending: "text-amber-400",
  closing: "text-amber-400",
  lifting: "text-emerald-400",
  dropping: "text-purple-400",
  returning_home: "text-zinc-400",
  attempt_done: "text-zinc-400",
  stopped: "text-zinc-500",
  halted: "text-red-400",
  done: "text-sky-400",
};

const STAGE_RESULT_COLOR: Record<StageResult, string> = {
  OK: "text-emerald-400",
  SPIKE: "text-amber-400",
  EMPTY: "text-orange-400",
  DROPPED: "text-red-400",
  SKIPPED: "text-zinc-600",
  FAIL: "text-red-500",
};

export function SelfPlayPanel(props: IDockviewPanelProps<object>) {
  const [objectType, setObjectType] = useState<ObjectType>(DEFAULT_OBJECT);
  const [prompt, setPrompt] = useState(OBJECT_PRESETS[DEFAULT_OBJECT].prompt);
  const [maxAttempts, setMaxAttempts] = useState(100);
  const { taskState, loading, run, stop } = useTask();
  const state = useSelfPlayStore((s) => s.state);
  const recent = useSelfPlayStore((s) => s.recentAttempts);

  const isActive =
    taskState.status === "running" || taskState.status === "paused";

  const handleObjectChange = useCallback(
    (e: React.ChangeEvent<HTMLSelectElement>) => {
      const t = e.target.value as ObjectType;
      setObjectType(t);
      setPrompt(OBJECT_PRESETS[t].prompt);
    },
    [],
  );

  const handleRun = useCallback(async () => {
    const trimmed = prompt.trim();
    if (!trimmed) return;
    await run({
      task: "self_play_pick",
      prompt: trimmed,
      max_attempts: maxAttempts,
      // 객체 type preset 의 raw 값. backend GripperSetup 으로 deserialize.
      gripper_setup: OBJECT_PRESETS[objectType].gripper,
    });
  }, [prompt, maxAttempts, objectType, run]);

  const successRate =
    state.stats.total > 0 ? (100 * state.stats.success) / state.stats.total : 0;

  return (
    <PanelShell
      icon={<Bot className="w-3.5 h-3.5" />}
      title="Self-play"
      panelId={props.api.id}
      api={props.api}
    >
      <Section label="Object Preset">
        <select
          value={objectType}
          onChange={handleObjectChange}
          disabled={isActive}
          className="w-full bg-zinc-900/80 border border-zinc-700/60 rounded px-2 py-1 text-[11px] font-mono text-zinc-100 focus:outline-none focus:border-zinc-500 disabled:opacity-50"
        >
          {(Object.keys(OBJECT_PRESETS) as ObjectType[]).map((k) => (
            <option key={k} value={k}>
              {OBJECT_PRESETS[k].label}
            </option>
          ))}
        </select>
        <div className="mt-1 text-[9px] font-mono text-zinc-500">
          close_current={OBJECT_PRESETS[objectType].gripper.close_current ?? "—"}
          mA · held≥
          {OBJECT_PRESETS[objectType].gripper.held_threshold ?? "—"}
        </div>
      </Section>

      <Section label="Prompt">
        <textarea
          value={prompt}
          onChange={(e) => setPrompt(e.target.value)}
          placeholder="e.g. white calibration cube"
          rows={2}
          disabled={isActive}
          className="w-full bg-zinc-900/80 border border-zinc-700/60 rounded px-2 py-1.5 text-[11px] font-mono text-zinc-100 placeholder:text-zinc-600 resize-none focus:outline-none focus:border-zinc-500 disabled:opacity-50"
        />
      </Section>

      <Section label="Max Attempts">
        <input
          type="number"
          min={1}
          max={10000}
          value={maxAttempts}
          onChange={(e) =>
            setMaxAttempts(Math.max(1, parseInt(e.target.value) || 1))
          }
          disabled={isActive}
          className="w-full bg-zinc-900/80 border border-zinc-700/60 rounded px-2 py-1 text-[11px] font-mono text-zinc-100 focus:outline-none focus:border-zinc-500 disabled:opacity-50"
        />
      </Section>

      <Section label="Control">
        {isActive ? (
          <button
            onClick={stop}
            className="w-full h-8 rounded bg-red-700 hover:bg-red-600 text-white text-[10px] font-mono uppercase tracking-wider flex items-center justify-center gap-1 transition-colors"
          >
            <Square className="w-3 h-3" />
            Stop
          </button>
        ) : (
          <button
            onClick={handleRun}
            disabled={loading || !prompt.trim()}
            className="w-full h-8 rounded bg-emerald-600 hover:bg-emerald-500 disabled:opacity-30 disabled:cursor-not-allowed text-white text-[10px] font-mono uppercase tracking-wider flex items-center justify-center gap-1 transition-colors"
          >
            <Play className="w-3 h-3" />
            {loading ? "..." : "Start"}
          </button>
        )}
      </Section>

      <Section label="Status">
        <div className="space-y-1 font-mono text-[10px]">
          <div className="flex justify-between">
            <span className="text-zinc-500">stage</span>
            <span className={`font-bold ${STAGE_COLOR[state.current_stage]}`}>
              {state.current_stage.toUpperCase()}
            </span>
          </div>
          <div className="flex justify-between">
            <span className="text-zinc-500">attempt</span>
            <span className="text-zinc-200">
              {state.attempt_id} / {state.max_attempts}
            </span>
          </div>
        </div>
      </Section>

      <Section label="Stats">
        <div className="grid grid-cols-2 gap-x-3 gap-y-1 font-mono text-[10px]">
          <div className="flex justify-between">
            <span className="text-zinc-500">total</span>
            <span className="text-zinc-200">{state.stats.total}</span>
          </div>
          <div className="flex justify-between">
            <span className="text-zinc-500">success</span>
            <span className="text-emerald-400">
              {state.stats.success} ({successRate.toFixed(0)}%)
            </span>
          </div>
          <div className="flex justify-between">
            <span className="text-zinc-500">s1 pass</span>
            <span className="text-zinc-200">{state.stats.s1_pass}</span>
          </div>
          <div className="flex justify-between">
            <span className="text-zinc-500">s2 pass</span>
            <span className="text-zinc-200">{state.stats.s2_pass}</span>
          </div>
          <div className="flex justify-between col-span-2">
            <span className="text-zinc-500">s3 pass</span>
            <span className="text-zinc-200">{state.stats.s3_pass}</span>
          </div>
        </div>
      </Section>

      {recent.length > 0 && (
        <Section label="Recent">
          <div className="space-y-0.5 font-mono text-[10px] max-h-40 overflow-y-auto">
            {recent.map((r) => (
              <div
                key={r.attempt_id}
                className="flex items-center gap-2 px-1 py-0.5 rounded hover:bg-zinc-900/40"
              >
                <span className="text-zinc-500 w-6 text-right">
                  #{r.attempt_id}
                </span>
                <span className={`w-6 ${STAGE_RESULT_COLOR[r.s1]}`}>
                  {r.s1}
                </span>
                <span className={`w-6 ${STAGE_RESULT_COLOR[r.s2]}`}>
                  {r.s2}
                </span>
                <span className={`w-6 ${STAGE_RESULT_COLOR[r.s3]}`}>
                  {r.s3}
                </span>
                {r.note && (
                  <span className="text-zinc-600 truncate text-[9px]">
                    {r.note}
                  </span>
                )}
              </div>
            ))}
          </div>
        </Section>
      )}
    </PanelShell>
  );
}
