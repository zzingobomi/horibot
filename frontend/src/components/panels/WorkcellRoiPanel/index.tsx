/**
 * WorkcellRoiPanel — 작업 셀 ROI 편집 UI (pnp_scenario_rework §9).
 *
 * SSOT = backend instance.yaml (shared_config 모듈 owner). 이 패널은:
 *   - 로드: SNAPSHOT_WORKCELL → store.saved (+ 안 만졌으면 draft 동기)
 *   - 갱신: WORKCELL_CHANGED 이벤트 → saved 반영 (다른 세션 저장이 실시간 표시.
 *     단 내 draft 가 dirty 면 draft 는 보호 — 편집 중 덮어쓰기 금지)
 *   - 편집: draft (숫자 입력 ↔ scenePart 면 핸들/중앙 화살표가 같은 사본)
 *   - 저장: **명시 Save** → SET_WORKCELL → yaml 반영 + detector Mirror 즉시 수렴.
 *     실패 = draft 유지 + 사유 표시 (침묵 fallback 금지).
 *
 * 3D 조작법 (scenePart): 면 drag = 그 면 resize / Shift+면 drag = 그 축 이동 /
 * 중앙 화살표 drag = 축 이동. 로봇 URDF 가 같은 씬에 있으므로 "로봇이 ROI 안에
 * 들어오면 로봇을 집으러 갈 수 있다" 는 눈으로 확인 (별도 가드 없음 — §9.1-3).
 */
import { useEffect, useState } from "react";
import { Button } from "@/components/ui/button";
import { useRobotId } from "@/hooks/useRobotId";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type { WorkcellRoi } from "@/api/generated/contract";
import { useWorkcellRoiStore, isRoiDirty } from "@/stores/workcellRoiStore";
import { FACE_AXIS, MIN_SPAN_M, type FaceId } from "./dragMath";

const BOUND_KEYS: FaceId[] = ["x_min", "x_max", "y_min", "y_max", "z_min", "z_max"];

// "ROI 만들기" 초기 draft — 로봇 앞 상식적 상자 (값은 바로 편집 대상일 뿐,
// 실측 보정은 사용자가 씬에서 드래그로).
const DEFAULT_ROI: WorkcellRoi = {
  x_min: 0.1,
  x_max: 0.35,
  y_min: -0.2,
  y_max: 0.2,
  z_min: -0.05,
  z_max: 0.3,
};

export function WorkcellRoiPanel() {
  const robotId = useRobotId();
  const snapshotSvc = useService(ServiceKey.SHAREDCONFIG_SNAPSHOT_WORKCELL, robotId);
  const setSvc = useService(ServiceKey.SHAREDCONFIG_SET_WORKCELL, robotId);
  const changed = useTopic(Topic.SHAREDCONFIG_WORKCELL_CHANGED);

  const draft = useWorkcellRoiStore((s) => s.drafts[robotId]);
  const saved = useWorkcellRoiStore((s) => s.saved[robotId]);
  const activeFace = useWorkcellRoiStore((s) => s.activeFace[robotId]);
  const setSaved = useWorkcellRoiStore((s) => s.setSaved);
  const setDraft = useWorkcellRoiStore((s) => s.setDraft);
  const revert = useWorkcellRoiStore((s) => s.revert);

  const [status, setStatus] = useState("");
  const [busy, setBusy] = useState(false);
  const dirty = isRoiDirty(draft, saved);

  // 최초 로드 — robot 별 1회 snapshot (이후 갱신은 CHANGED 이벤트가 담당)
  useEffect(() => {
    let cancelled = false;
    (async () => {
      const res = await snapshotSvc.call({});
      if (cancelled) return;
      if (!res.success) {
        setStatus(`ROI 로드 실패: ${res.message} — shared_config 모듈 확인`);
        return;
      }
      const roi = res.data?.robots?.[robotId];
      if (roi) setSaved(robotId, roi);
    })();
    return () => {
      cancelled = true;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [robotId]);

  // 서버측 변경 (다른 세션/클라이언트 저장) — saved 반영. dirty draft 는 store 가 보호.
  useEffect(() => {
    if (changed && changed.robot_id === robotId) {
      setSaved(robotId, changed.roi);
    }
  }, [changed, robotId, setSaved]);

  const setBound = (key: FaceId, raw: string) => {
    if (!draft) return;
    const v = Number.parseFloat(raw);
    if (!Number.isFinite(v)) return;
    const axis = FACE_AXIS[key];
    const [lo, hi] = [`${axis}_min`, `${axis}_max`] as [FaceId, FaceId];
    const next = { ...draft, [key]: v };
    // min/max 역전 방지 — backend validator 가 거부하는 값을 애초에 못 만들게
    if (key === lo) next[lo] = Math.min(v, draft[hi] - MIN_SPAN_M);
    else next[hi] = Math.max(v, draft[lo] + MIN_SPAN_M);
    setDraft(robotId, next);
  };

  const save = async () => {
    if (!draft) return;
    setBusy(true);
    setStatus("");
    try {
      const res = await setSvc.call({ robot_id: robotId, roi: draft });
      if (res.success && res.data) {
        setSaved(robotId, res.data.roi);
        setDraft(robotId, res.data.roi);
        setStatus("저장됨 — instance.yaml 반영 + detector 즉시 적용");
      } else {
        // draft 유지 — 사용자가 수정 후 재시도 가능 (탈출구)
        setStatus(`저장 실패: ${res.message}`);
      }
    } catch (e) {
      setStatus(`저장 오류: ${e instanceof Error ? e.message : String(e)}`);
    } finally {
      setBusy(false);
    }
  };

  const field = (key: FaceId) => (
    <label
      key={key}
      className={`flex items-center gap-1 ${activeFace === key ? "text-amber-400" : ""}`}
    >
      <span className="w-12 font-mono text-muted-foreground">{key}</span>
      <input
        type="number"
        step={0.01}
        value={draft ? Math.round(draft[key] * 1000) / 1000 : 0}
        onChange={(e) => setBound(key, e.target.value)}
        data-testid={`roi-${key}`}
        className={`min-w-0 flex-1 rounded border bg-zinc-900 px-1 py-0.5 font-mono ${
          activeFace === key ? "border-amber-500" : "border-zinc-700"
        }`}
      />
    </label>
  );

  if (!draft) {
    return (
      <div
        className="flex h-full flex-col items-center justify-center gap-3 p-3 text-[12px]"
        data-testid="workcell-roi-panel"
      >
        <p className="text-muted-foreground">
          {snapshotSvc.pending
            ? "ROI 로드 중…"
            : "이 robot 은 workcell ROI 미설정 — 검출 셀 컷이 꺼져 있습니다"}
        </p>
        {!snapshotSvc.pending && (
          <Button
            size="sm"
            onClick={() => setDraft(robotId, DEFAULT_ROI)}
            data-testid="roi-create"
          >
            ROI 만들기
          </Button>
        )}
        {status && <p className="font-mono text-red-400">{status}</p>}
      </div>
    );
  }

  return (
    <div
      className="flex h-full flex-col gap-3 overflow-y-auto p-3 text-[12px]"
      data-testid="workcell-roi-panel"
    >
      <section>
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          workcell ROI (base frame, m)
        </div>
        <p className="mb-2 text-[10px] text-muted-foreground">
          씬에서 직접: 면 drag=크기 · Shift+면/중앙 화살표 drag=이동. 로봇이 상자
          안에 들어오면 로봇을 물체로 오검출할 수 있습니다.
        </p>
        <div className="grid grid-cols-2 gap-x-3 gap-y-1">
          {BOUND_KEYS.map(field)}
        </div>
      </section>

      <section className="flex items-center gap-2">
        <Button
          size="sm"
          onClick={save}
          disabled={!dirty || busy}
          data-testid="roi-save"
        >
          {busy ? "저장 중…" : "저장"}
        </Button>
        <Button
          size="sm"
          variant="ghost"
          onClick={() => revert(robotId)}
          disabled={!dirty || busy || !saved}
          data-testid="roi-revert"
        >
          되돌리기
        </Button>
        {dirty && (
          <span className="font-mono text-amber-400" data-testid="roi-dirty">
            미저장 변경
          </span>
        )}
      </section>

      <div className="mt-auto font-mono text-muted-foreground" data-testid="roi-status">
        {status ||
          (dirty
            ? "저장 전엔 backend 에 반영되지 않습니다"
            : saved
              ? "저장된 ROI 표시 중"
              : "새 ROI — 저장하면 instance.yaml 에 기록")}
      </div>
    </div>
  );
}
