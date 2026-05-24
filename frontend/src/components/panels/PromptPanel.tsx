import { useCallback, useState } from "react";
import { Sparkles, Play, Square } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { bridge } from "@/api/bridge";
import { ServiceKey } from "@/constants/topics";
import { PanelShell } from "@/components/canvas/ui/PanelShell";
import { Section } from "@/components/canvas/ui/Section";
import { useTask } from "@/hooks/useTask";
import { useDetectorStore } from "@/store/detectorStore";

const DEFAULT_PROMPT = "black car key";

/**
 * 자연어 prompt + Run/Stop 컨트롤.
 *
 *  - Run: pick_named_object task 실행 (prompt 인자).
 *  - Detect: omx/perception/grounded_detect 단독 호출 → backend 가
 *    `PERCEPTION_GROUNDED_STATE` 토픽으로 자동 broadcast → useBridge 가
 *    detectorStore.groundedResult 갱신. 여기선 service 호출만, 결과 처리 X.
 */
export function PromptPanel(props: IDockviewPanelProps<object>) {
  const [prompt, setPrompt] = useState(DEFAULT_PROMPT);
  const [detecting, setDetecting] = useState(false);
  const setGroundedResult = useDetectorStore((s) => s.setGroundedResult);
  const { taskState, loading, run, stop } = useTask();

  const isActive =
    taskState.status === "running" || taskState.status === "paused";

  const handleRun = useCallback(async () => {
    const trimmed = prompt.trim();
    if (!trimmed) return;
    // 새 task 시작 — 이전 detect 결과 클리어 (시작 시점에 깨끗한 상태로).
    setGroundedResult(null);
    await run({ task: "pick_named_object", prompt: trimmed });
  }, [prompt, run, setGroundedResult]);

  const handleDetect = useCallback(async () => {
    const trimmed = prompt.trim();
    if (!trimmed) return;
    setDetecting(true);
    try {
      // CPU 추론 + 첫 호출 시 모델 로드(~수십초) 포함. 넉넉히.
      const res = await bridge.callService(
        ServiceKey.PERCEPTION_GROUNDED_DETECT,
        { prompt: trimmed },
        { timeoutMs: 60000 },
      );
      // 성공 시: backend 가 토픽 broadcast → useBridge 가 store update. 여기선 X.
      // 실패 시: 토픽 broadcast 안 옴 → 이전 결과 명시적 클리어.
      if (!res.success) {
        setGroundedResult(null);
      }
    } finally {
      setDetecting(false);
    }
  }, [prompt, setGroundedResult]);

  return (
    <PanelShell
      icon={<Sparkles className="w-3.5 h-3.5" />}
      title="Prompt"
      panelId={props.api.id}
      api={props.api}
    >
      <Section label="Object Description">
        <textarea
          value={prompt}
          onChange={(e) => setPrompt(e.target.value)}
          placeholder="e.g. black car key on the table"
          rows={3}
          disabled={isActive}
          className="w-full bg-zinc-900/80 border border-zinc-700/60 rounded px-2 py-1.5 text-[11px] font-mono text-zinc-100 placeholder:text-zinc-600 resize-none focus:outline-none focus:border-zinc-500 disabled:opacity-50"
        />
      </Section>

      <Section label="Control">
        <div className="flex gap-1.5">
          <button
            onClick={handleDetect}
            disabled={isActive || detecting || !prompt.trim()}
            title="Detection 만 단독 호출 (debug)"
            className="flex-1 h-8 rounded bg-sky-700/80 hover:bg-sky-600 disabled:opacity-30 disabled:cursor-not-allowed text-white text-[10px] font-mono uppercase tracking-wider transition-colors"
          >
            {detecting ? "..." : "Detect"}
          </button>
          {isActive ? (
            <button
              onClick={stop}
              className="flex-1 h-8 rounded bg-red-700 hover:bg-red-600 text-white text-[10px] font-mono uppercase tracking-wider flex items-center justify-center gap-1 transition-colors"
            >
              <Square className="w-3 h-3" />
              Stop
            </button>
          ) : (
            <button
              onClick={handleRun}
              disabled={loading || !prompt.trim()}
              className="flex-1 h-8 rounded bg-emerald-600 hover:bg-emerald-500 disabled:opacity-30 disabled:cursor-not-allowed text-white text-[10px] font-mono uppercase tracking-wider flex items-center justify-center gap-1 transition-colors"
            >
              <Play className="w-3 h-3" />
              {loading ? "..." : "Run"}
            </button>
          )}
        </div>
      </Section>
    </PanelShell>
  );
}
