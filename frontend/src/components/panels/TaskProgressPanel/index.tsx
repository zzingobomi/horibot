/**
 * TaskProgressPanel — task 진행 표시 + 디버거 (task 공통 부품).
 *
 * TRACE (step 진입 누적 — label/depth/status/detail, 중첩은 depth 들여쓰기) +
 * STATE (status/current_name/error/breakpoints) 로 진행을 그린다.
 *
 * trace 가 비어 있을 땐 PREVIEW (정적 프리뷰 — backend 가 시나리오 **소스만
 * 읽어** 뽑은 step 구조, 실행 0) 를 같은 flat+depth 렌더로 보여준다 — 실행 전에
 * breakpoint/run_to 대상을 고를 수 있게. 프리뷰는 "존재하는 구조"지 실행 보장이
 * 아니다: 조건부(if)/반복(loop)은 배지로 표시만 하고, <동적> 노드는 정적으로 못
 * 푼 호출 자리 (실행이 시작되면 trace 가 실제 진입으로 채운다 — 자연 치환).
 *
 * 디버거: ● dot 클릭 = TOGGLE_BREAKPOINT(name) (실행 전 프리뷰에서 미리 박기
 * 포함 — runner 가 run 간 보존 + run 밖 토글도 STATE 로 즉시 보임) / RUNNING
 * 에서 [일시정지] / PAUSED 에서 [재개]·[한 스텝]·entry 별 [▶ 여기까지]
 * (run-to-cursor). 실패 = error 박스에 사유 (backend 가 "다음 행동" 까지 담아
 * 보냄 — 침묵 금지).
 */
import { useEffect } from "react";
import { Button } from "@/components/ui/button";
import { useBridgeConnected, useService, useStream } from "@/framework";
import { useTaskRobots } from "@/hooks/useTaskRobots";
import {
  ServiceKey,
  TaskStatus,
  Topic,
  type PreviewEntry,
  type TraceEntry,
} from "@/api/generated/contract";

const STATUS_COLOR: Record<string, string> = {
  [TaskStatus.RUNNING]: "text-sky-400",
  [TaskStatus.SUCCESS]: "text-emerald-400",
  [TaskStatus.FAILED]: "text-red-400",
  [TaskStatus.PAUSED]: "text-amber-400",
  [TaskStatus.STOPPED]: "text-zinc-400",
  [TaskStatus.IDLE]: "text-zinc-500",
};

const ENTRY_DOT: Record<string, string> = {
  running: "bg-sky-400",
  completed: "bg-emerald-400",
  failed: "bg-red-400",
};

export function TaskProgressPanel() {
  // task 참여 robot (계약 조회) — 로드 전 undefined = 스트림 미확장 (데이터 없음).
  const robotId = useTaskRobots(ServiceKey.PICKANDPLACE_LIST_ROBOTS)[0];
  const state = useStream(Topic.PICKANDPLACE_STATE, { robotId });
  const trace = useStream(Topic.PICKANDPLACE_TRACE, { robotId });

  const pauseSvc = useService(ServiceKey.PICKANDPLACE_PAUSE, robotId);
  const resumeSvc = useService(ServiceKey.PICKANDPLACE_RESUME, robotId);
  const stepOnceSvc = useService(ServiceKey.PICKANDPLACE_STEP_ONCE, robotId);
  const runToSvc = useService(ServiceKey.PICKANDPLACE_RUN_TO, robotId);
  const toggleBpSvc = useService(ServiceKey.PICKANDPLACE_TOGGLE_BREAKPOINT, robotId);

  // 정적 프리뷰 — mount 시 1회 fetch (시나리오는 backend 재시작 전까지 불변,
  // 캐시에 있으면 재호출 없음). timestamp 로 "시도 완료" 를 판정해 실패 시
  // 자동 재시도 폭주를 막고, 실패는 사유 + [재시도] 로 표시 (침묵 금지).
  const connected = useBridgeConnected();
  const previewSvc = useService(ServiceKey.PICKANDPLACE_PREVIEW);
  const previewCall = previewSvc.call;
  useEffect(() => {
    if (!connected || previewSvc.pending || previewSvc.timestamp !== 0) return;
    void previewCall({});
  }, [connected, previewSvc.pending, previewSvc.timestamp, previewCall]);

  const st = state.value;
  const entries: TraceEntry[] = trace.value?.entries ?? [];
  const status = st?.status ?? TaskStatus.IDLE;
  const currentName = st?.current_name ?? "";
  const breakpoints = new Set(st?.breakpoints ?? []);
  const running = status === TaskStatus.RUNNING;
  const paused = status === TaskStatus.PAUSED;

  // trace 없음 = 아직 안 돌았음 → 정적 프리뷰가 그 자리를 채운다 (실행이
  // 시작되면 trace 가 실제 진입으로 치환 — <동적>/조건부가 확정 이름으로).
  const preview: PreviewEntry[] = previewSvc.data?.entries ?? [];
  const showPreview = entries.length === 0;

  return (
    <div
      className="flex h-full flex-col gap-2 overflow-y-auto p-3 text-[12px]"
      data-testid="task-progress-panel"
    >
      <div className="flex items-center gap-2">
        <span className="font-mono uppercase text-muted-foreground">status</span>
        <span
          className={`font-mono font-semibold ${STATUS_COLOR[status] ?? "text-zinc-400"}`}
          data-testid="task-status"
        >
          {status}
        </span>
        {st?.task_name && (
          <span className="truncate font-mono text-muted-foreground">
            · {st.task_name}
          </span>
        )}
        {paused && currentName && (
          <span className="truncate font-mono text-amber-400">
            ⏸ {st?.current_title || currentName} 직전
          </span>
        )}
      </div>

      {/* 디버거 컨트롤 — VSCode 등가 (pause / resume / step over) */}
      <div className="flex flex-wrap gap-2">
        <Button
          size="sm"
          variant="outline"
          disabled={!running}
          onClick={() => void pauseSvc.call({})}
          data-testid="task-pause"
        >
          일시정지
        </Button>
        <Button
          size="sm"
          variant="outline"
          disabled={!paused}
          onClick={() => void resumeSvc.call({})}
          data-testid="task-resume"
        >
          재개
        </Button>
        <Button
          size="sm"
          variant="outline"
          disabled={!paused}
          onClick={() => void stepOnceSvc.call({})}
          data-testid="task-step-once"
        >
          한 스텝
        </Button>
      </div>

      {st?.error && (
        <div
          className="rounded border border-red-800/60 bg-red-950/30 p-2 font-mono text-red-300"
          data-testid="task-error"
        >
          {st.error}
        </div>
      )}

      <div className="font-mono uppercase text-muted-foreground">
        {showPreview ? `미리보기 (${preview.length})` : `trace (${entries.length})`}
      </div>
      {showPreview && (
        <span className="text-[10px] text-muted-foreground">
          실행 전 정적 구조 — 조건부/반복은 실행 시 결정. ● 클릭 = breakpoint
          미리 설정 (실행하면 실제 진입이 이 자리를 채움).
        </span>
      )}
      <div className="flex flex-col gap-1" data-testid="task-entries">
        {showPreview ? (
          previewSvc.pending || previewSvc.timestamp === 0 ? (
            <span className="text-muted-foreground">미리보기 불러오는 중…</span>
          ) : !previewSvc.success ? (
            <div
              className="flex items-center gap-2 rounded border border-red-800/60 bg-red-950/30 p-2 font-mono text-red-300"
              data-testid="task-preview-error"
            >
              <span className="flex-1 truncate">
                미리보기 실패: {previewSvc.message || "알 수 없는 오류"}
              </span>
              <Button
                size="sm"
                variant="outline"
                onClick={() => void previewCall({})}
                data-testid="task-preview-retry"
              >
                재시도
              </Button>
            </div>
          ) : preview.length === 0 ? (
            <span className="text-muted-foreground">표시할 step 없음</span>
          ) : (
            preview.map((e, i) => {
              const hasBp = breakpoints.has(e.name);
              return (
                <div
                  key={`${i}:${e.name}`}
                  className="flex items-center gap-2 rounded border border-zinc-800 px-2 py-1"
                  style={{ marginLeft: (e.depth ?? 0) * 14 }}
                  data-testid="task-preview-entry"
                >
                  {e.dynamic ? (
                    // <동적> = breakpoint 대상 아님 (이름 미확정) — dot 없이 자리 표식
                    <span
                      className="h-3 w-3 shrink-0 rounded-full border border-dashed border-zinc-500"
                      title="정적으로 대상을 못 푼 호출 — 실행하면 실제 step 이 여기 나타남"
                    />
                  ) : (
                    <button
                      type="button"
                      onClick={() => void toggleBpSvc.call({ name: e.name })}
                      title="브레이크포인트 토글 (실행 전 미리 박기)"
                      data-testid="task-preview-bp"
                      className={`h-3 w-3 shrink-0 rounded-full bg-zinc-600 ${
                        hasBp ? "ring-2 ring-red-500" : "hover:ring-2 hover:ring-zinc-500"
                      }`}
                    />
                  )}
                  <span className="flex-1 truncate font-mono">
                    {e.dynamic ? e.name : e.title || e.name}
                    {(e.dynamic ? e.title : e.title && e.name) && (
                      <span className="ml-1 text-[10px] text-muted-foreground">
                        {e.dynamic ? e.title : e.name}
                      </span>
                    )}
                  </span>
                  {e.conditional && (
                    <span className="rounded bg-zinc-800 px-1 font-mono text-[10px] text-amber-300/80">
                      조건부
                    </span>
                  )}
                  {e.repeated && (
                    <span className="rounded bg-zinc-800 px-1 font-mono text-[10px] text-sky-300/80">
                      반복
                    </span>
                  )}
                  {e.recursive && (
                    <span className="rounded bg-zinc-800 px-1 font-mono text-[10px] text-purple-300/80">
                      재귀
                    </span>
                  )}
                  {e.unavailable && (
                    <span
                      className="rounded bg-zinc-800 px-1 font-mono text-[10px] text-zinc-400"
                      title="소스 획득 불가 — 하위 step 미상"
                    >
                      소스 없음
                    </span>
                  )}
                </div>
              );
            })
          )
        ) : (
          entries.map((e, i) => {
            const hasBp = breakpoints.has(e.name);
            const isCurrent = e.name === currentName;
            return (
              <div
                key={`${i}:${e.name}`}
                className={`flex items-center gap-2 rounded border px-2 py-1 ${
                  isCurrent ? "border-sky-500" : "border-zinc-700"
                }`}
                // 중첩 step — depth 들여쓰기 (wire 는 flat 리스트 + depth,
                // 트리 표현은 UI 몫. 접기/펼치기는 후속 polish).
                style={{ marginLeft: (e.depth ?? 0) * 14 }}
                data-testid="task-entry"
              >
                {/* dot = 상태색. 클릭 = breakpoint toggle (빨간 ring) — 같은
                    step name 은 다음 run 에서도 유효 (runner 가 run 간 보존). */}
                <button
                  type="button"
                  onClick={() => void toggleBpSvc.call({ name: e.name })}
                  title="브레이크포인트 토글"
                  data-testid="task-entry-bp"
                  className={`h-3 w-3 shrink-0 rounded-full ${ENTRY_DOT[e.status] ?? "bg-zinc-600"} ${
                    hasBp ? "ring-2 ring-red-500" : "hover:ring-2 hover:ring-zinc-500"
                  }`}
                />
                {/* title = 표시 문구 (한글 등), name = 식별자 — title 있으면
                    title 이 주(主), name 은 작은 보조 (breakpoint 대상 확인용) */}
                <span className="flex-1 truncate font-mono">
                  {e.title || e.name}
                  {e.title && (
                    <span className="ml-1 text-[10px] text-muted-foreground">
                      {e.name}
                    </span>
                  )}
                </span>
                {e.detail && (
                  <span
                    className={`max-w-[40%] truncate font-mono text-[10px] ${
                      e.status === "failed" ? "text-red-400" : "text-muted-foreground"
                    }`}
                    title={e.detail}
                  >
                    {e.detail}
                  </span>
                )}
                {paused && (
                  <button
                    type="button"
                    onClick={() => void runToSvc.call({ name: e.name })}
                    title="여기까지 실행 (run to cursor)"
                    data-testid="task-run-to"
                    className="font-mono text-[10px] text-zinc-500 hover:text-sky-400"
                  >
                    ▶│
                  </button>
                )}
                <span className="font-mono text-[10px] text-muted-foreground">
                  {e.status}
                </span>
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}
