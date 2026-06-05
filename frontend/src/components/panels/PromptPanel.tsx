import { useCallback, useEffect, useRef, useState } from "react";
import { Sparkles, Play, Square, Eye } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { useBridgeConnected, useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { useDetectorOverride } from "@/domain/stores/detector";
import { PanelShell } from "@/components/shared/PanelShell";
import { Section } from "@/components/shared/Section";
import { defaultTaskState, type TaskState } from "@/types/task";

const DEFAULT_PROMPT = "흰 큐브 들어서 파란 박스에 놔";

export function PromptPanel(props: IDockviewPanelProps<object>) {
  const [prompt, setPrompt] = useState(DEFAULT_PROMPT);
  const bridgeConnected = useBridgeConnected();
  const taskState =
    (useTopic(Topic.TASK_STATE) as TaskState | null) ?? defaultTaskState;
  const runSvc = useService(ServiceKey.TASK_RUN);
  const stopSvc = useService(ServiceKey.TASK_STOP);
  const previewSvc = useService(ServiceKey.TASK_PREVIEW);
  const detectSvc = useService(ServiceKey.PERCEPTION_GROUNDED_DETECT);
  const hideDetections = useDetectorOverride((s) => s.hide);

  const isActive =
    taskState.status === "running" || taskState.status === "paused";

  const handlePreview = useCallback(async () => {
    const trimmed = prompt.trim();
    if (!trimmed) return;
    // 응답은 backend 가 TASK_TREE 토픽으로 broadcast → useTopic 이 즉시 반영.
    await previewSvc.call({ task: "pick_and_place", prompt: trimmed });
  }, [prompt, previewSvc]);

  // bridge 연결된 후 1회 자동 preview — 앱 켜자마자 default prompt 의 트리가 보임.
  const didInitialPreview = useRef(false);
  useEffect(() => {
    if (!bridgeConnected || didInitialPreview.current) return;
    didInitialPreview.current = true;
    void handlePreview();
  }, [bridgeConnected, handlePreview]);

  const handleRun = useCallback(async () => {
    const trimmed = prompt.trim();
    if (!trimmed) return;
    hideDetections(); // 새 task 시작 — 이전 detect 결과 가림
    await runSvc.call({ task: "pick_and_place", prompt: trimmed });
  }, [prompt, runSvc, hideDetections]);

  const handleDetect = useCallback(async () => {
    const trimmed = prompt.trim();
    if (!trimmed) return;
    const res = await detectSvc.call(
      { prompt: trimmed },
      { timeoutMs: 60000 },
    );
    // 성공 = backend 가 topic broadcast → useTopic 자동 반영.
    // 실패 = topic broadcast 안 옴 → 이전 결과 명시적 가림.
    if (!res.success) hideDetections();
  }, [prompt, detectSvc, hideDetections]);

  return (
    <PanelShell
      icon={<Sparkles className="w-3.5 h-3.5" />}
      title="Prompt"
      panelId={props.api.id}
      api={props.api}
    >
      <Section label="Natural language command">
        <textarea
          value={prompt}
          onChange={(e) => setPrompt(e.target.value)}
          placeholder="예: 흰 큐브 들어서 파란 박스에 놔 / pick white cube and place on blue box"
          rows={3}
          disabled={isActive}
          className="w-full bg-zinc-900/80 border border-zinc-700/60 rounded px-2 py-1.5 text-[11px] font-mono text-zinc-100 placeholder:text-zinc-600 resize-none focus:outline-none focus:border-zinc-500 disabled:opacity-50"
        />
      </Section>

      <Section label="Control">
        <div className="flex gap-1.5">
          <button
            onClick={() => void handleDetect()}
            disabled={isActive || detectSvc.pending || !prompt.trim()}
            title="prompt 그대로 grounded detect (debug, LLM parse 없음)"
            className="flex-1 h-8 rounded bg-sky-700/80 hover:bg-sky-600 disabled:opacity-30 disabled:cursor-not-allowed text-white text-[10px] font-mono uppercase tracking-wider transition-colors"
          >
            {detectSvc.pending ? "..." : "Detect"}
          </button>
          <button
            onClick={() => void handlePreview()}
            disabled={isActive || previewSvc.pending || !prompt.trim()}
            title="task 트리만 미리 보기 (실행 X). breakpoint 사전 박기용"
            className="flex-1 h-8 rounded bg-zinc-700 hover:bg-zinc-600 disabled:opacity-30 disabled:cursor-not-allowed text-white text-[10px] font-mono uppercase tracking-wider flex items-center justify-center gap-1 transition-colors"
          >
            <Eye className="w-3 h-3" />
            {previewSvc.pending ? "..." : "Preview"}
          </button>
          {isActive ? (
            <button
              onClick={() => void stopSvc.call({})}
              className="flex-1 h-8 rounded bg-red-700 hover:bg-red-600 text-white text-[10px] font-mono uppercase tracking-wider flex items-center justify-center gap-1 transition-colors"
            >
              <Square className="w-3 h-3" />
              Stop
            </button>
          ) : (
            <button
              onClick={() => void handleRun()}
              disabled={runSvc.pending || !prompt.trim()}
              className="flex-1 h-8 rounded bg-emerald-600 hover:bg-emerald-500 disabled:opacity-30 disabled:cursor-not-allowed text-white text-[10px] font-mono uppercase tracking-wider flex items-center justify-center gap-1 transition-colors"
            >
              <Play className="w-3 h-3" />
              {runSvc.pending ? "..." : "Run"}
            </button>
          )}
        </div>
      </Section>
    </PanelShell>
  );
}
