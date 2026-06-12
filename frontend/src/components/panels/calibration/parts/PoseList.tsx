import type { PerPoseResidual, PoseMeta } from "./types";

/**
 * 자세별 weight indicator — 사용자가 *어느 자세가 outlier 자동 처리되었는지* 한눈에.
 * 수치 (residual / weight 숫자) 는 노출 X — 색깔 dot + tooltip 만.
 *
 *   - 회색 dot   = 정상 (weight ≥ 0.7 또는 정보 없음)
 *   - 노란 dot   = 낮음 (0.3 ≤ weight < 0.7) — 자동 down-weight 진행 중
 *   - 빨강 dot   = 자동 제외 (weight < 0.3 또는 1차 outlier 제거됨)
 */
function PoseStatusDot({ res }: { res: PerPoseResidual | null }) {
  if (!res) return <span className="w-1.5 h-1.5 rounded-full bg-zinc-700/40" />;
  if (res.excluded) {
    return (
      <span
        className="w-1.5 h-1.5 rounded-full bg-red-500"
        title="자동 제외됨 — outlier 감지"
      />
    );
  }
  if (res.weight !== null && res.weight < 0.3) {
    return (
      <span
        className="w-1.5 h-1.5 rounded-full bg-red-500"
        title="자동 제외 의심 — 다음 캘 시 다시 시도 권장"
      />
    );
  }
  if (res.weight !== null && res.weight < 0.7) {
    return (
      <span
        className="w-1.5 h-1.5 rounded-full bg-amber-500"
        title="weight 낮음 — 자동 down-weight 적용 중"
      />
    );
  }
  return (
    <span
      className="w-1.5 h-1.5 rounded-full bg-emerald-500"
      title="정상"
    />
  );
}

export function HandEyePoseList({
  poses,
  perPose,
}: {
  poses: PoseMeta[];
  perPose?: PerPoseResidual[] | null;
}) {
  if (poses.length === 0) {
    return (
      <p className="text-[11px] text-zinc-500 italic font-mono">
        포즈 없음 — 첫 자세를 캡처하세요.
      </p>
    );
  }

  // id → PerPoseResidual 매핑 (compute 결과의 per_pose_residual).
  const perPoseMap = new Map<number, PerPoseResidual>();
  if (perPose) for (const r of perPose) perPoseMap.set(r.id, r);

  return (
    <div className="rounded border border-zinc-800/60 bg-zinc-900/40 p-2 max-h-48 overflow-y-auto">
      <div className="text-[10px] text-zinc-500 font-mono mb-1 px-1 uppercase tracking-wide">
        {poses.length}개 누적
      </div>
      <ul className="space-y-0.5">
        {poses.map((p) => {
          const ts = new Date(p.timestamp * 1000);
          const time = ts.toLocaleTimeString("ko-KR", { hour12: false });
          const res = perPoseMap.get(p.id) ?? null;
          return (
            <li
              key={p.id}
              className="flex items-center gap-2 px-2 py-1 rounded text-[11px] font-mono hover:bg-zinc-800/40 text-zinc-300"
            >
              <PoseStatusDot res={res} />
              <span className="text-zinc-500 w-8 tabular-nums">#{p.id}</span>
              <span className="flex-1 tabular-nums">{time}</span>
            </li>
          );
        })}
      </ul>
    </div>
  );
}
