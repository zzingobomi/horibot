/**
 * Hand-Eye calibration panel — Phase 1/2 분기.
 *
 * **Phase 1 (manualModeActive=true)**: 사용자가 8장 손으로 자유 자세 캡처.
 * 카메라 + ChArUco overlay 의 한 장 단위 hint (검출 + tilt) 만. σ / 추천 hide.
 * [자동 추천 시작] 버튼 — n>=8 enabled.
 *
 * **Phase 2 (manualModeActive=false)**: [자동 추천 시작] 누르면 multi-start BA
 * 자동 호출 → manualModeActive=false → 추천 자세 + σ + saturate 알림 + 명시 신호
 * UI 등장. 사용자는 추천 [이동] / [캡처] / [COMMIT] 반복.
 *
 * 라이브 카메라는 [CalibrationCameraPanel] 이 공유 — 본 패널은 컨트롤 + 추천 + 결과만.
 */
import { Crosshair } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { useParams } from "react-router-dom";
import { PanelShell } from "@/components/shared/PanelShell";
import { PanelButton } from "@/components/shared/PanelButton";
import { Section } from "@/components/shared/Section";
import { NextPoseCard } from "./parts/NextPoseCard";
import { HandEyePoseList } from "./parts/PoseList";
import { useCalibrationStore } from "@/domain/stores/calibration";
import { useCalibrationResults } from "@/hooks/useCalibrationResults";
import type {
  AxisDistribution,
  CalibThresholds,
  CoachVerdict,
  HandeyeSaturateState,
  HandEyeSigmaState,
} from "./parts/types";

/**
 * verdict → 색 / 라벨 분기. 4 상태:
 *   good              → 초록 "충분"
 *   narrow_sigma_good → 노랑 "σ 좋음, 자세 부족"  ← σ 만 보면 trap, 별도 라벨 필수
 *   needs_work        → 노랑 "경계"
 *   bad               → 빨강 "부족"
 */
function verdictMeta(verdict: CoachVerdict | null): {
  color: string;
  bgColor: string;
  borderColor: string;
  label: string;
} {
  switch (verdict) {
    case "good":
      return {
        color: "text-green-400",
        bgColor: "bg-green-500/10",
        borderColor: "border-green-500/40",
        label: "충분 — COMMIT 권장",
      };
    case "narrow_sigma_good":
      return {
        color: "text-amber-400",
        bgColor: "bg-amber-500/10",
        borderColor: "border-amber-500/40",
        label: "σ 통과, 자세 다양성 부족 — COMMIT 가능 / 추가 캡처 권장",
      };
    case "needs_work":
      return {
        color: "text-amber-400",
        bgColor: "bg-amber-500/10",
        borderColor: "border-amber-500/40",
        label: "경계 — 추가 캡처 권장",
      };
    case "bad":
      return {
        color: "text-red-400",
        bgColor: "bg-red-500/10",
        borderColor: "border-red-500/40",
        label: "부족 — 캡처 절차 / setup 점검",
      };
    default:
      return {
        color: "text-zinc-500",
        bgColor: "bg-zinc-800/40",
        borderColor: "border-zinc-700/40",
        label: "측정 중…",
      };
  }
}

/**
 * σ live badge — n<trusted 면 회색.
 */
function LiveSigmaBadge({
  sigma,
  thresholds,
}: {
  sigma: HandEyeSigmaState | null;
  thresholds: CalibThresholds | null;
}) {
  if (!sigma || sigma.sigma_rot_deg === null || sigma.sigma_t_mm === null) {
    return null;
  }
  const trustedN = thresholds?.min_poses_for_trusted_sigma ?? 8;
  const trusted = sigma.pose_count >= trustedN;

  const rotColor =
    !trusted || thresholds === null
      ? "text-zinc-500"
      : sigma.sigma_rot_deg < thresholds.sigma_rot_good_deg
        ? "text-green-500"
        : sigma.sigma_rot_deg < thresholds.sigma_rot_warn_deg
          ? "text-amber-500"
          : "text-red-500";
  const tColor =
    !trusted || thresholds === null
      ? "text-zinc-500"
      : sigma.sigma_t_mm < thresholds.sigma_t_good_mm
        ? "text-green-500"
        : sigma.sigma_t_mm < thresholds.sigma_t_warn_mm
          ? "text-amber-500"
          : "text-red-500";
  return (
    <div
      className="text-[10px] font-mono flex items-center gap-1.5"
      title={
        trusted
          ? `auto-BA (n=${sigma.pose_count}, ${sigma.ba_mode ?? "?"})`
          : `n=${sigma.pose_count} < ${trustedN} — BA DOF 흡수 부족, σ 신뢰도 낮음`
      }
    >
      <span className={rotColor}>{sigma.sigma_rot_deg.toFixed(2)}°</span>
      <span className={tColor}>{sigma.sigma_t_mm.toFixed(1)}mm</span>
      <span className="text-zinc-500">
        n={sigma.pose_count}
        {!trusted && " ↓"}
      </span>
    </div>
  );
}

/**
 * verdict 배너 — 색깔 + 라벨. σ 만 보면 trap (narrow_sigma_good) 자리를 별도 색깔로
 * 분리 표시. trauma source 의 root cause fix UI 측 핵심 자리.
 */
function VerdictBanner({ verdict }: { verdict: CoachVerdict | null }) {
  if (!verdict) return null;
  const meta = verdictMeta(verdict);
  return (
    <div
      className={`rounded border ${meta.borderColor} ${meta.bgColor} ${meta.color} px-2 py-1.5 text-[11px] font-mono leading-snug`}
    >
      {meta.label}
    </div>
  );
}

/**
 * 자세 다양성 표 — axis 별 std vs threshold. low_diversity 자리 강조.
 * narrow_sigma_good verdict 시 사용자가 *어느 axis 변주* 해야 할지 직접 보임.
 */
function AxisDistributionTable({
  axes,
}: {
  axes: AxisDistribution[] | undefined;
}) {
  if (!axes || axes.length === 0) return null;
  return (
    <div className="flex flex-col gap-0.5 font-mono text-[10px]">
      <div className="flex justify-between text-zinc-500 uppercase tracking-wide px-1">
        <span>축</span>
        <span>σ / 권장</span>
      </div>
      {axes.map((d) => {
        const color = d.is_low_diversity ? "text-amber-400" : "text-zinc-400";
        return (
          <div
            key={d.motor_id}
            className={`flex justify-between px-1 ${color}`}
            title={d.suggestion_text}
          >
            <span>
              J{d.motor_id}
              {d.is_low_diversity && " ⚠"}
            </span>
            <span className="tabular-nums">
              {d.std_deg.toFixed(1)}° / {d.threshold_deg.toFixed(0)}°
            </span>
          </div>
        );
      })}
    </div>
  );
}

/**
 * COMMIT 섹션 — verdict 기반 안내 + 버튼 gate.
 *   - bad 만 disable (σ 진짜 못 씀)
 *   - good / narrow_sigma_good / needs_work 다 enable (사용자 판단권)
 *   - verdict 별 안내 문구 다름 (σ self-consistency 의 한계 명시)
 */
function CommitSection({
  verdict,
  hasResult,
  stale,
  loading,
  onCommit,
}: {
  verdict: CoachVerdict | null;
  hasResult: boolean;
  stale: boolean;
  loading: boolean;
  onCommit: () => Promise<void>;
}) {
  const disabledByVerdict = verdict === "bad";
  const disabled = loading || !hasResult || stale || disabledByVerdict;
  const guide =
    verdict === "good"
      ? "σ 통과 + 자세 다양성 충족 — hand_eye.npz 저장."
      : verdict === "narrow_sigma_good"
        ? "σ 통과지만 자세 다양성 부족. 저장 가능하지만 부족 axis 변주로 추가 캡처 후 재평가 권장."
        : verdict === "needs_work"
          ? "σ 경계 — 추가 캡처 권장. 그래도 저장 가능."
          : verdict === "bad"
            ? "σ 부족 — 저장 비활성. 캡처 절차 / setup 점검."
            : "σ 측정 후 저장 가능.";
  return (
    <div className="flex flex-col gap-2">
      <p className="text-[11px] text-zinc-500 font-mono leading-snug">{guide}</p>
      <PanelButton
        variant="secondary"
        onClick={() => void onCommit()}
        disabled={disabled}
      >
        COMMIT (저장)
      </PanelButton>
    </div>
  );
}

/**
 * Saturate 알림 — σ 변화율 거의 0 → "saturate" 명시. in_good=true 면 COMMIT 권장,
 * false 면 floor 도달 escape 안내.
 */
function SaturateBanner({
  saturate,
}: {
  saturate: HandeyeSaturateState | null;
}) {
  if (!saturate || !saturate.saturate) return null;
  const color = saturate.in_good
    ? "border-green-500/50 bg-green-500/10 text-green-300"
    : "border-amber-500/50 bg-amber-500/10 text-amber-300";
  return (
    <div className={`rounded border ${color} px-2 py-1.5 text-[11px]`}>
      {saturate.reason}
    </div>
  );
}

export function HandEyePanel(props: IDockviewPanelProps<object>) {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const { refetch: refetchCalibrationResults } = useCalibrationResults(robotId);

  const poses = useCalibrationStore((s) => s.poses);
  const liveSigma = useCalibrationStore((s) => s.liveSigma);
  const compute = useCalibrationStore((s) => s.compute);
  const computeStale = useCalibrationStore((s) => s.computeStale);
  const recommendations = useCalibrationStore((s) => s.recommendations);
  const noCandidatesReason = useCalibrationStore((s) => s.noCandidatesReason);
  const visited = useCalibrationStore((s) => s.visited);
  const activeIndex = useCalibrationStore((s) => s.activeIndex);
  const thresholds = useCalibrationStore((s) => s.thresholds);
  const saturate = useCalibrationStore((s) => s.saturate);
  const manualModeActive = useCalibrationStore((s) => s.manualModeActive);
  const loading = useCalibrationStore((s) => s.loading);
  const status = useCalibrationStore((s) => s.status);

  const captureAction = useCalibrationStore((s) => s.capture);
  const resetAction = useCalibrationStore((s) => s.reset);
  const commitAction = useCalibrationStore((s) => s.commit);
  const movedAction = useCalibrationStore((s) => s.moved);
  const exitManualMode = useCalibrationStore((s) => s.exitManualMode);
  const reportFail = useCalibrationStore((s) => s.reportFail);

  const minManualPoses = thresholds?.min_poses_for_trusted_sigma ?? 8;
  const canExitManual = poses.length >= minManualPoses;

  const handleReset = async () => {
    if (!confirm("누적된 모든 포즈를 삭제합니다. 계속할까요?")) return;
    await resetAction();
  };

  const handleExitManual = async () => {
    if (!canExitManual) return;
    await exitManualMode();
  };

  const handleCommit = async () => {
    const res = await commitAction();
    if (res.success) {
      await refetchCalibrationResults();
    }
  };

  return (
    <PanelShell
      icon={<Crosshair className="w-3.5 h-3.5" />}
      title="Hand-Eye"
      panelId={props.api.id}
      api={props.api}
      expandedHeight={640}
    >
      {manualModeActive ? (
        // ──── Phase 1: 수동 자유 자세 캡처 ────
        <>
          <Section label={`Capture — 수동 (${poses.length}/${minManualPoses})`}>
            <div className="flex flex-col gap-2">
              <p className="text-[11px] text-zinc-500 leading-snug font-mono">
                자세 손으로 잡고 [캡처] {minManualPoses}장. 다양하게 (J1 yaw / J4
                pitch / J5 roll 골고루). overlay 초록일 때 캡처.
              </p>
              <div className="flex gap-2">
                <PanelButton
                  variant="primary"
                  className="flex-1"
                  onClick={() => void captureAction()}
                  disabled={loading}
                >
                  {loading ? "..." : "캡처"}
                </PanelButton>
                <PanelButton
                  variant="outline"
                  onClick={() => void handleReset()}
                  disabled={loading || poses.length === 0}
                >
                  리셋
                </PanelButton>
              </div>
              <HandEyePoseList poses={poses} />
            </div>
          </Section>

          <Section label="다음">
            <div className="flex flex-col gap-2">
              <p className="text-[11px] text-zinc-500 leading-snug font-mono">
                {canExitManual
                  ? `${poses.length}장 누적 — 다음 단계로 진행하세요.`
                  : `${minManualPoses - poses.length}장 더 캡처해주세요.`}
              </p>
              <PanelButton
                variant="primary"
                onClick={() => void handleExitManual()}
                disabled={!canExitManual || loading}
              >
                {loading ? "계산 중..." : "자동 추천 시작"}
              </PanelButton>
            </div>
          </Section>
        </>
      ) : (
        // ──── Phase 2: 자동 추천 모드 ────
        <>
          <Section label="Status">
            <div className="flex flex-col gap-2">
              <div className="flex items-center justify-between">
                <span className="text-[11px] text-zinc-500">
                  누적 {poses.length}장
                </span>
                <LiveSigmaBadge sigma={liveSigma} thresholds={thresholds} />
              </div>
              <VerdictBanner verdict={liveSigma?.coach_verdict ?? null} />
              <AxisDistributionTable axes={liveSigma?.axis_distributions} />
              <SaturateBanner saturate={saturate} />
            </div>
          </Section>

          <Section label="Next Pose">
            <NextPoseCard
              recommendations={recommendations}
              noCandidatesReason={noCandidatesReason}
              visited={visited}
              activeIndex={activeIndex}
              onMoved={movedAction}
              onReportFail={reportFail}
              disabled={loading}
            />
          </Section>

          <Section label="Capture">
            <div className="flex flex-col gap-2">
              <div className="flex gap-2">
                <PanelButton
                  variant="primary"
                  className="flex-1"
                  onClick={() => void captureAction()}
                  disabled={loading}
                >
                  {loading ? "..." : "캡처"}
                </PanelButton>
                <PanelButton
                  variant="outline"
                  onClick={() => void handleReset()}
                  disabled={loading || poses.length === 0}
                >
                  리셋
                </PanelButton>
              </div>
              <HandEyePoseList poses={poses} />
            </div>
          </Section>

          <Section label="Commit">
            <CommitSection
              verdict={liveSigma?.coach_verdict ?? null}
              hasResult={!!(compute || liveSigma)}
              stale={computeStale}
              loading={loading}
              onCommit={handleCommit}
            />
          </Section>
        </>
      )}

      {status && (
        <Section label="Message">
          <p className="text-[11px] text-zinc-400 leading-snug font-mono">
            {status}
          </p>
        </Section>
      )}
    </PanelShell>
  );
}
