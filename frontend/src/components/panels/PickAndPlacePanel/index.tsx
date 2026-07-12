/**
 * PickAndPlacePanel — Pick & Place 실행 컨트롤 (task 페이지 코어).
 *
 * 흐름: typed 폼 (pick/place prompt — RunRequest 필드 그대로) 을 직접 채우거나,
 * 자연어 → [파싱] (LLM_PARSE_COMMAND) 으로 폼을 채운 뒤 → [실행] PICKANDPLACE_RUN.
 * 옛 PREVIEW 단계는 소멸 — 파싱 결과가 폼에 보이고 사용자가 수정/확인 후 실행.
 * [중지] = PICKANDPLACE_STOP (in-flight 모션 즉시 끊김 + 모터 정지 — runner 계약).
 *
 * 대상 robot 은 task 가 선언 (GET /tasks robot_ids) — useTaskRobotId. 서비스는
 * task 모듈 소유 (robot-agnostic 키) 지만 캐시 정체성 규약대로 robotId 를 전달.
 */
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { useService } from "@/framework";
import { useTaskRobotId } from "@/hooks/useTasks";
import { ServiceKey } from "@/api/generated/contract";

const TASK_NAME = "pick_and_place";

export function PickAndPlacePanel() {
  // 미로드 시 "" — 실행 버튼 gate (robot 바인딩을 모르면 실행 안 함).
  const robotId = useTaskRobotId(TASK_NAME) ?? "";

  const parseSvc = useService(ServiceKey.LLM_PARSE_COMMAND);
  const runSvc = useService(ServiceKey.PICKANDPLACE_RUN, robotId);
  const stopSvc = useService(ServiceKey.PICKANDPLACE_STOP, robotId);

  // 개발 중 반복 입력 절감 — 대표 시나리오 기본값.
  const [text, setText] = useState("흰색 작고 네모난 큐브를 파란 상자에 둬");
  const [pickObject, setPickObject] = useState("");
  const [placeObject, setPlaceObject] = useState("");
  const [msg, setMsg] = useState("");
  const [busy, setBusy] = useState(false);

  const onParse = async () => {
    const cmd = text.trim();
    if (!cmd) return;
    setBusy(true);
    setMsg("파싱 중…");
    const res = await parseSvc.call({ text: cmd });
    setBusy(false);
    const d = res.data as {
      ok?: boolean;
      parsed?: { pick_object: string; place_object: string | null } | null;
      message?: string;
    } | null;
    if (d?.ok && d.parsed) {
      setPickObject(d.parsed.pick_object);
      setPlaceObject(d.parsed.place_object ?? "");
      setMsg("파싱 결과 확인/수정 후 [실행]");
    } else {
      setMsg(`파싱 실패: ${d?.message ?? res.message}`);
    }
  };

  const onRun = async () => {
    const pick = pickObject.trim();
    if (!pick) return;
    const res = await runSvc.call({
      pick_object: pick,
      place_object: placeObject.trim(),
    });
    const d = res.data as { accepted?: boolean; message?: string } | null;
    setMsg(
      d?.accepted
        ? "실행 시작 — 진행은 Task Progress"
        : `거부: ${d?.message ?? res.message}`,
    );
  };

  const onStop = async () => {
    const res = await stopSvc.call({});
    const d = res.data as { ok?: boolean; message?: string } | null;
    setMsg(d?.ok ? "중지 요청 (모션 정지)" : `중지 실패: ${d?.message ?? res.message}`);
  };

  return (
    <div
      className="flex h-full flex-col gap-3 overflow-y-auto p-3 text-[12px]"
      data-testid="pnp-panel"
    >
      <section>
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          자연어 명령 (선택)
        </div>
        <textarea
          value={text}
          onChange={(e) => setText(e.target.value)}
          placeholder='예: "흰색 작고 네모난 큐브를 파란 상자에 둬"'
          data-testid="pnp-input"
          rows={2}
          className="w-full resize-none rounded border border-zinc-700 bg-zinc-900 px-2 py-1 font-mono"
        />
        <div className="mt-1">
          <Button
            size="sm"
            variant="outline"
            onClick={onParse}
            disabled={busy || !text.trim()}
            data-testid="pnp-parse"
          >
            파싱 → 폼 채우기
          </Button>
        </div>
      </section>

      <section>
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          실행 param (RunRequest)
        </div>
        <label className="mb-1 block">
          <span className="text-muted-foreground">pick (필수) </span>
          <input
            value={pickObject}
            onChange={(e) => setPickObject(e.target.value)}
            placeholder="예: white small cube"
            data-testid="pnp-pick"
            className="mt-0.5 w-full rounded border border-zinc-700 bg-zinc-900 px-2 py-1 font-mono"
          />
        </label>
        <label className="block">
          <span className="text-muted-foreground">place (빈 값 = 집기만) </span>
          <input
            value={placeObject}
            onChange={(e) => setPlaceObject(e.target.value)}
            placeholder="예: blue box"
            data-testid="pnp-place"
            className="mt-0.5 w-full rounded border border-zinc-700 bg-zinc-900 px-2 py-1 font-mono"
          />
        </label>
        <div className="mt-2 flex gap-2">
          <Button
            size="sm"
            onClick={onRun}
            disabled={!pickObject.trim() || !robotId}
            data-testid="pnp-run"
          >
            실행
          </Button>
          <Button size="sm" variant="ghost" onClick={onStop} data-testid="pnp-stop">
            중지
          </Button>
        </div>
      </section>

      <div className="text-muted-foreground" data-testid="pnp-msg">
        {msg}
      </div>
    </div>
  );
}
