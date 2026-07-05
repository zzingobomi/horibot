/**
 * PromptPanel — 자연어 명령 → LLM 파싱 → PnP 실행 (§17 NL PnP, RobotTaskMode 코어).
 *
 * 흐름: 사용자가 "흰색 큐브를 파란 상자에 둬" 입력 → [파싱] LLM_PARSE_COMMAND →
 * (pick/place) 확인 표시 → [실행] TASK_RUN(pick_and_place, params). 파싱 결과를
 * 사용자가 보고 실행하는 2-스텝 (LLM 오해 시 확인 가능). 진행 상황은 TaskProgressPanel.
 *
 * detector/llm/task 는 robot-agnostic — LLM parse 는 robot 무관, TASK_RUN 은 req 에
 * robot_id (docs/backend_v2.md §2.7). 정확도(검출/파싱)는 실물 hardware tuning (§17.5).
 */
import { useState } from "react";
import { useParams } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { DEFAULT_ROBOT_ID } from "@/constants";
import { useService } from "@/framework";
import { ServiceKey } from "@/api/generated/contract";

interface Parsed {
  pick_object: string;
  place_object: string | null;
}

export function PromptPanel() {
  const { id } = useParams<{ id: string }>();
  const robotId = id ?? DEFAULT_ROBOT_ID;

  const parseSvc = useService(ServiceKey.LLM_PARSE_COMMAND);
  const runSvc = useService(ServiceKey.TASK_RUN, robotId);
  const stopSvc = useService(ServiceKey.TASK_STOP, robotId);

  const [text, setText] = useState("");
  const [parsed, setParsed] = useState<Parsed | null>(null);
  const [msg, setMsg] = useState("");
  const [busy, setBusy] = useState(false);

  const onParse = async () => {
    const cmd = text.trim();
    if (!cmd) return;
    setBusy(true);
    setMsg("파싱 중…");
    setParsed(null);
    const res = await parseSvc.call({ text: cmd });
    setBusy(false);
    const d = res.data as
      | { ok?: boolean; parsed?: Parsed | null; message?: string }
      | null;
    if (d?.ok && d.parsed) {
      setParsed(d.parsed);
      setMsg("");
    } else {
      setMsg(`파싱 실패: ${d?.message ?? res.message}`);
    }
  };

  const onRun = async () => {
    if (!parsed) return;
    const params: Record<string, string> = { pick_object: parsed.pick_object };
    if (parsed.place_object) params.place_object = parsed.place_object;
    const res = await runSvc.call({
      robot_id: robotId,
      task_name: "pick_and_place",
      params,
    });
    const d = res.data as { accepted?: boolean; message?: string } | null;
    setMsg(d?.accepted ? "실행 시작 — 진행은 Task Progress" : `거부: ${d?.message ?? res.message}`);
  };

  const onStop = async () => {
    await stopSvc.call({ robot_id: robotId });
    setMsg("중지 요청");
  };

  return (
    <div
      className="flex h-full flex-col gap-3 overflow-y-auto p-3 text-[12px]"
      data-testid="prompt-panel"
    >
      <section>
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          자연어 명령
        </div>
        <textarea
          value={text}
          onChange={(e) => setText(e.target.value)}
          placeholder='예: "흰색 큐브를 파란 상자에 둬"'
          data-testid="prompt-input"
          rows={2}
          className="w-full resize-none rounded border border-zinc-700 bg-zinc-900 px-2 py-1 font-mono"
        />
        <div className="mt-2 flex gap-2">
          <Button
            size="sm"
            onClick={onParse}
            disabled={busy || !text.trim()}
            data-testid="prompt-parse"
          >
            파싱
          </Button>
          <Button
            size="sm"
            variant="outline"
            onClick={onRun}
            disabled={!parsed}
            data-testid="prompt-run"
          >
            실행
          </Button>
          <Button
            size="sm"
            variant="ghost"
            onClick={onStop}
            data-testid="prompt-stop"
          >
            중지
          </Button>
        </div>
      </section>

      {parsed && (
        <section
          className="rounded border border-emerald-700/60 bg-emerald-950/30 p-2 font-mono"
          data-testid="prompt-parsed"
        >
          <div>
            <span className="text-muted-foreground">pick </span>
            {parsed.pick_object}
          </div>
          <div>
            <span className="text-muted-foreground">place </span>
            {parsed.place_object ?? "(없음 — 집기만)"}
          </div>
        </section>
      )}

      <div className="text-muted-foreground" data-testid="prompt-msg">
        {msg}
      </div>
    </div>
  );
}
