/**
 * HandoverPanel — handover(양팔 인계) 실행 컨트롤 (task 페이지 코어).
 *
 * RunRequest 필드 그대로의 typed 폼: pick(봉 프롬프트) / place(빈 값 = 수취 후
 * 적치 생략) / **stop_before_receive** (omx 집기+제시까지만 — 실물 검증 프로토콜:
 * 먼저 이걸 켜고 omx 쪽을 눈으로 확인한 뒤 꺼서 so101 수취를 붙인다,
 * docs/omx_handover_realtest_handoff.md). 기본값 ON — 실물 미검증 task 의 첫
 * 런이 바로 풀 시퀀스로 달리는 사고 방지.
 *
 * [중지] = HANDOVER_STOP (in-flight 모션 즉시 끊김 + 참여 robot 전원 STOP —
 * runner 계약). 대상 robot = task 바인딩 계약 조회 (useTaskRobots) — 서비스는
 * robot-agnostic 키지만 캐시 정체성 규약대로 robotId 를 전달.
 */
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { useService } from "@/framework";
import { useTaskRobots } from "@/hooks/useTaskRobots";
import { ServiceKey } from "@/api/generated/contract";

export function HandoverPanel() {
  // 로드 전 undefined → [실행] disabled (아래 !robotId 게이트).
  const robots = useTaskRobots(ServiceKey.HANDOVER_LIST_ROBOTS);
  const robotId = robots[0];

  const runSvc = useService(ServiceKey.HANDOVER_RUN, robotId);
  const stopSvc = useService(ServiceKey.HANDOVER_STOP, robotId);

  // 봉(8×2×2cm 주황 각봉) 기본 프롬프트 — 개발/실물 반복 입력 절감.
  const [pickObject, setPickObject] = useState("orange block");
  const [placeObject, setPlaceObject] = useState("");
  const [stopBeforeReceive, setStopBeforeReceive] = useState(true);
  const [msg, setMsg] = useState("");

  const onRun = async () => {
    const pick = pickObject.trim();
    if (!pick) return;
    const res = await runSvc.call({
      pick_object: pick,
      place_object: placeObject.trim(),
      stop_before_receive: stopBeforeReceive,
    });
    const d = res.data as { accepted?: boolean; message?: string } | null;
    setMsg(
      d?.accepted
        ? stopBeforeReceive
          ? "실행 시작 (omx 집기+제시까지) — 진행은 Task Progress"
          : "실행 시작 — 진행은 Task Progress"
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
      data-testid="handover-panel"
    >
      <section>
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          참여 robot (giver → receiver)
        </div>
        <div className="font-mono text-muted-foreground" data-testid="handover-robots">
          {robots.length ? robots.slice().reverse().join(" → ") : "로드 중…"}
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
            placeholder="예: orange block"
            data-testid="handover-pick"
            className="mt-0.5 w-full rounded border border-zinc-700 bg-zinc-900 px-2 py-1 font-mono"
          />
        </label>
        <label className="block">
          <span className="text-muted-foreground">place (빈 값 = 수취까지만) </span>
          <input
            value={placeObject}
            onChange={(e) => setPlaceObject(e.target.value)}
            placeholder="예: white box"
            data-testid="handover-place"
            className="mt-0.5 w-full rounded border border-zinc-700 bg-zinc-900 px-2 py-1 font-mono"
          />
        </label>
        <label className="mt-2 flex items-start gap-2">
          <input
            type="checkbox"
            checked={stopBeforeReceive}
            onChange={(e) => setStopBeforeReceive(e.target.checked)}
            data-testid="handover-stop-before-receive"
            className="mt-0.5"
          />
          <span>
            <span className="font-mono">stop_before_receive</span>
            <span className="block text-[10px] text-muted-foreground">
              omx 집기+제시까지만 하고 종료 (실물 검증 1단계 — 제시 자세를 눈으로
              확인 후 해제하고 so101 수취를 붙이세요)
            </span>
          </span>
        </label>
        <div className="mt-2 flex gap-2">
          <Button
            size="sm"
            onClick={onRun}
            disabled={!pickObject.trim() || !robotId}
            data-testid="handover-run"
          >
            실행
          </Button>
          <Button size="sm" variant="ghost" onClick={onStop} data-testid="handover-stop">
            중지
          </Button>
        </div>
      </section>

      <div className="text-muted-foreground" data-testid="handover-msg">
        {msg}
      </div>
    </div>
  );
}
