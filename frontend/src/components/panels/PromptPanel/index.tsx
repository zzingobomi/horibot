/**
 * PromptPanel — 자연어 명령 → LLM 파싱 → PnP 실행 (§17 NL PnP, RobotTaskMode 코어).
 *
 * 흐름 (v1 디버거 플로우 계승): [파싱] LLM_PARSE_COMMAND → (pick/place) 확인 표시
 * + **자동 TASK_PREVIEW** (실행 없이 step tree publish → TaskProgressPanel 에 목록,
 * 거기서 브레이크포인트 미리 박음) → [실행] TASK_RUN. 파싱 결과를 사용자가 보고
 * 실행하는 2-스텝 (LLM 오해 시 확인 가능).
 *
 * detector/llm/task 는 robot-agnostic — LLM parse 는 robot 무관, TASK_RUN 은 req 에
 * robot_id (docs/backend.md §2.7). 정확도(검출/파싱)는 실물 hardware tuning (§17.5).
 */
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { useService } from "@/framework";
import { useTaskRobotId } from "@/hooks/useTasks";
import { ServiceKey } from "@/api/generated/contract";

// 이 패널이 다루는 task — 대상 robot 은 backend task 바인딩(useTaskRobotId)에서 옴
// (ambient default 로봇 아님). 추후 task 메뉴 도입 시 라우트 param 으로 승격.
const TASK_NAME = "pick_and_place";

interface Parsed {
  pick_object: string;
  place_object: string | null;
}

/** parsed → TASK_RUN/PREVIEW 공용 params (한 곳). */
function taskParams(p: Parsed): Record<string, string> {
  const params: Record<string, string> = { pick_object: p.pick_object };
  if (p.place_object) params.place_object = p.place_object;
  return params;
}

export function PromptPanel() {
  // 대상 robot = task 가 선언한 robot (backend GET /tasks). 미로드 시 "" — 버튼 gate.
  const robotId = useTaskRobotId(TASK_NAME) ?? "";

  const parseSvc = useService(ServiceKey.LLM_PARSE_COMMAND);
  const previewSvc = useService(ServiceKey.TASK_PREVIEW, robotId);
  const runSvc = useService(ServiceKey.TASK_RUN, robotId);
  const stopSvc = useService(ServiceKey.TASK_STOP, robotId);

  // 개발 중 반복 입력 절감 — 대표 시나리오를 기본값으로 (§17.5 tuning 단계).
  const [text, setText] = useState("흰색 작고 네모난 큐브를 파란 상자에 둬");
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
    const d = res.data as {
      ok?: boolean;
      parsed?: Parsed | null;
      message?: string;
    } | null;
    if (d?.ok && d.parsed) {
      setParsed(d.parsed);
      // v1 플로우 — 파싱 즉시 preview 로 step tree publish (실행 X).
      // TaskProgressPanel 에 목록이 뜨고 거기서 브레이크포인트를 미리 박는다.
      const pv = await previewSvc.call({
        robot_id: robotId,
        task_name: TASK_NAME,
        params: taskParams(d.parsed),
      });
      const pd = pv.data as { ok?: boolean; message?: string } | null;
      setMsg(
        pd?.ok
          ? "step 목록 확인 (Task Progress) — 브레이크포인트 설정 후 [실행]"
          : `preview 실패: ${pd?.message ?? pv.message}`,
      );
    } else {
      setMsg(`파싱 실패: ${d?.message ?? res.message}`);
    }
  };

  const onRun = async () => {
    if (!parsed) return;
    const res = await runSvc.call({
      robot_id: robotId,
      task_name: "pick_and_place",
      params: taskParams(parsed),
    });
    const d = res.data as { accepted?: boolean; message?: string } | null;
    setMsg(
      d?.accepted
        ? "실행 시작 — 진행은 Task Progress"
        : `거부: ${d?.message ?? res.message}`,
    );
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
          placeholder='예: "흰색 작고 네모난 큐브를 파란 상자에 둬"'
          data-testid="prompt-input"
          rows={2}
          className="w-full resize-none rounded border border-zinc-700 bg-zinc-900 px-2 py-1 font-mono"
        />
        <div className="mt-2 flex gap-2">
          <Button
            size="sm"
            onClick={onParse}
            disabled={busy || !text.trim() || !robotId}
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
