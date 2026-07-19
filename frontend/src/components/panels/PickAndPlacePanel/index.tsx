/**
 * PickAndPlacePanel — Pick & Place 실행 컨트롤 (task 페이지 코어).
 *
 * 흐름: typed 폼 (pick/place prompt — RunRequest 필드 그대로) 을 직접 채우거나,
 * 자연어 → [파싱] (LLM_PARSE_COMMAND) 으로 폼을 채운 뒤 → [실행] PICKANDPLACE_RUN.
 * 옛 PREVIEW 단계는 소멸 — 파싱 결과가 폼에 보이고 사용자가 수정/확인 후 실행.
 * [중지] = PICKANDPLACE_STOP (in-flight 모션 즉시 끊김 + 모터 정지 — runner 계약).
 *
 * 대상 robot = task 바인딩 계약 조회 (useTaskRobots). 서비스는
 * task 모듈 소유 (robot-agnostic 키) 지만 캐시 정체성 규약대로 robotId 를 전달.
 */
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { useService } from "@/framework";
import { useTaskRobots } from "@/hooks/useTaskRobots";
import { ServiceKey } from "@/api/generated/contract";
import {
  DEFAULT_BUILD_VOXEL_M,
  useScanStore,
  VOXEL_TIERS,
} from "@/stores/scanStore";

// build_world 마지막 선택 기억 — 매 실행마다 다시 켤 필요 없게 (기본 off =
// "빨리 픽앤플레이스만" 이 기본, 2026-07-18 UX 결정).
const BUILD_WORLD_LS_KEY = "pnp.buildWorld";
// 월드 빌드 voxel 마지막 선택 (m) — build_world 와 독립 기억.
const WORLD_VOXEL_LS_KEY = "pnp.worldVoxelM";

/** 재구성 생성 시각 → "N분/시간/일 전" (stale 월드 침묵 금지 라벨). */
function agoLabel(iso: string): string {
  const ms = Date.now() - new Date(iso).getTime();
  if (!Number.isFinite(ms) || ms < 0) return "";
  const min = Math.floor(ms / 60_000);
  if (min < 1) return "방금";
  if (min < 60) return `${min}분 전`;
  const hr = Math.floor(min / 60);
  if (hr < 24) return `${hr}시간 전`;
  return `${Math.floor(hr / 24)}일 전`;
}

export function PickAndPlacePanel() {
  // 로드 전 undefined → [실행] disabled (아래 !robotId 게이트).
  const robotId = useTaskRobots(ServiceKey.PICKANDPLACE_LIST_ROBOTS)[0];

  const parseSvc = useService(ServiceKey.LLM_PARSE_COMMAND);
  const runSvc = useService(ServiceKey.PICKANDPLACE_RUN, robotId);
  const stopSvc = useService(ServiceKey.PICKANDPLACE_STOP, robotId);

  // 개발 중 반복 입력 절감 — 대표 시나리오 기본값.
  const [text, setText] = useState("흰색 작고 네모난 큐브를 파란 상자에 둬");
  const [pickObject, setPickObject] = useState("");
  const [placeObject, setPlaceObject] = useState("");
  const [msg, setMsg] = useState("");
  const [busy, setBusy] = useState(false);

  // World 토글 2종 (독립): 갱신 = run 옵션(pose 당 +1~2s 비용, 기본 off) /
  // 표시 = 씬 게이트(비용 0, 기본 on — scanStore workcell 전역).
  const [buildWorld, setBuildWorldState] = useState(
    () => localStorage.getItem(BUILD_WORLD_LS_KEY) === "1",
  );
  const setBuildWorld = (on: boolean) => {
    setBuildWorldState(on);
    localStorage.setItem(BUILD_WORLD_LS_KEY, on ? "1" : "0");
  };
  const [worldVoxelM, setWorldVoxelMState] = useState(() => {
    const v = Number(localStorage.getItem(WORLD_VOXEL_LS_KEY));
    return VOXEL_TIERS.some((t) => t.m === v) ? v : DEFAULT_BUILD_VOXEL_M;
  });
  const setWorldVoxelM = (m: number) => {
    setWorldVoxelMState(m);
    localStorage.setItem(WORLD_VOXEL_LS_KEY, String(m));
  };
  const worldVisible = useScanStore((s) => s.worldVisible);
  const setWorldVisible = useScanStore((s) => s.setWorldVisible);
  const meshMeta = useScanStore((s) => s.meshMeta);

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
      build_world: buildWorld,
      // 갱신 off 여도 명시 전송 (계약 필드) — backend 는 build_world=false 면 무시.
      world_voxel_size: worldVoxelM,
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
        <label className="mt-2 flex items-center gap-2">
          <input
            type="checkbox"
            checked={buildWorld}
            onChange={(e) => setBuildWorld(e.target.checked)}
            data-testid="pnp-build-world"
          />
          <span>
            search 중 월드 갱신 (스캔)
            <span className="text-muted-foreground"> — pose 당 +1~2s</span>
          </span>
        </label>
        {buildWorld && (
          <label className="mt-1 ml-6 flex items-center gap-2">
            <span className="text-muted-foreground">품질 (voxel)</span>
            <select
              value={worldVoxelM}
              onChange={(e) => setWorldVoxelM(Number(e.target.value))}
              data-testid="pnp-world-voxel"
              className="rounded border border-zinc-700 bg-zinc-900 px-1 py-0.5 font-mono"
            >
              {VOXEL_TIERS.map((t) => (
                <option key={t.m} value={t.m}>
                  {t.label}
                </option>
              ))}
            </select>
          </label>
        )}
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

      <section>
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          world (배경 메시)
        </div>
        <label className="flex items-center gap-2">
          <input
            type="checkbox"
            checked={worldVisible}
            onChange={(e) => setWorldVisible(e.target.checked)}
            data-testid="pnp-world-visible"
          />
          <span>월드 표시</span>
        </label>
        <div className="mt-1 text-muted-foreground" data-testid="pnp-world-label">
          {meshMeta?.createdAt
            ? `현재 월드: ${agoLabel(meshMeta.createdAt)} 스캔 · ` +
              `${meshMeta.vertexCount.toLocaleString()} verts` +
              (meshMeta.voxelSizeM
                ? ` · ${(meshMeta.voxelSizeM * 1000).toFixed(0)}mm`
                : "")
            : "월드 없음 — Scan 모드 또는 위 '월드 갱신' 옵션으로 생성"}
        </div>
      </section>

      <div className="text-muted-foreground" data-testid="pnp-msg">
        {msg}
      </div>
    </div>
  );
}
