/**
 * Calibration Capture Panel — Phase 1/2 분기.
 *
 * **Phase 1 (manualModeActive=true)**: 사용자가 8장 손으로 자유 자세 캡처.
 * 카메라 + ChArUco overlay 의 한 장 단위 hint (검출 + tilt) 만. σ / 추천 hide.
 * [수동 모드 종료] 버튼 — n>=8 enabled.
 *
 * **Phase 2 (manualModeActive=false)**: [수동 모드 종료] 누르면 multi-start BA
 * 자동 호출 → manualModeActive=false → 추천 자세 + σ + saturate 알림 + 명시 신호
 * UI 등장. 사용자는 추천 [이동] / [캡처] / [COMMIT] 반복.
 */
import { Camera } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { useParams } from "react-router-dom";
import { CameraFeed } from "@/components/shared/CameraFeed";
import { Button } from "@/components/ui/button";
import { PanelShell } from "@/components/shared/PanelShell";
import { Section } from "@/components/shared/Section";
import { CalibJointBar } from "@/components/panels/CalibrationActionsPanel/JointBar";
import { CheckerboardOverlay } from "@/components/panels/CalibrationActionsPanel/CheckerboardOverlay";
import { NextPoseCard } from "@/components/panels/CalibrationActionsPanel/NextPoseCard";
import { HandEyePoseList } from "@/components/panels/CalibrationActionsPanel/PoseList";
import { useCalibrationStore } from "@/domain/stores/calibration";
import { useCalibrationResults } from "@/hooks/useCalibrationResults";
import type {
  CalibThresholds,
  HandeyeSaturateState,
  HandEyeSigmaState,
} from "@/components/panels/CalibrationActionsPanel/types";

/**
 * σ live badge — Phase 2 자리 자취 자리 자체 자리 자취 자리 색깔 활성. n<trusted 면 회색.
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
 * Saturate 알림 — σ 변화율 거의 0 → "saturate" 명시. in_good=true 면 COMMIT 권장,
 * false 면 floor 도달 escape 안내. 사용자 외부 도구 진입 자체 자리 자취 자리 막음.
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

export function CalibrationCapturePanel(props: IDockviewPanelProps<object>) {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const { refetch: refetchCalibrationResults } = useCalibrationResults(robotId);

  const preview = useCalibrationStore((s) => s.preview);
  const previewStale = useCalibrationStore((s) => s.previewStale);
  const poses = useCalibrationStore((s) => s.poses);
  const liveSigma = useCalibrationStore((s) => s.liveSigma);
  const compute = useCalibrationStore((s) => s.compute);
  const computeStale = useCalibrationStore((s) => s.computeStale);
  const recommendations = useCalibrationStore((s) => s.recommendations);
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
      icon={<Camera className="w-3.5 h-3.5" />}
      title="Calib Capture"
      panelId={props.api.id}
      api={props.api}
      expandedHeight={720}
    >
      {/* 카메라 자리 자취 자리 — 두 phase 공통 자체 자리 자취 자리 */}
      <Section label="Camera">
        <CameraFeed
          className="w-full aspect-video"
          overlay={
            <>
              <CalibJointBar />
              <CheckerboardOverlay preview={preview} stale={previewStale} />
            </>
          }
        />
      </Section>

      {manualModeActive ? (
        // ──── Phase 1: 수동 자유 자세 캡처 ────
        <>
          <Section label={`Capture — 수동 (${poses.length}/${minManualPoses})`}>
            <div className="flex flex-col gap-2">
              <p className="text-[11px] text-zinc-500 leading-snug">
                자세 손으로 잡고 [캡처] {minManualPoses}장. 다양하게 (J1 yaw / J4
                pitch / J5 roll 골고루). overlay 초록일 때 캡처.
              </p>
              <div className="flex gap-2">
                <Button
                  size="sm"
                  className="flex-1"
                  onClick={() => void captureAction()}
                  disabled={loading}
                >
                  {loading ? "..." : "캡처"}
                </Button>
                <Button
                  size="sm"
                  variant="outline"
                  onClick={() => void handleReset()}
                  disabled={loading || poses.length === 0}
                >
                  리셋
                </Button>
              </div>
              <HandEyePoseList poses={poses} />
            </div>
          </Section>

          <Section label="자동 모드 진입">
            <div className="flex flex-col gap-2">
              <p className="text-[11px] text-zinc-500 leading-snug">
                {canExitManual
                  ? `${poses.length}장 누적 — 이제 자동 추천 모드 진입 가능.`
                  : `${minManualPoses - poses.length}장 더 캡처 후 자동 추천 활성.`}
              </p>
              <Button
                size="sm"
                onClick={() => void handleExitManual()}
                disabled={!canExitManual || loading}
              >
                {loading ? "Multi-start BA..." : "수동 모드 종료 → 자동 추천"}
              </Button>
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
              <SaturateBanner saturate={saturate} />
            </div>
          </Section>

          <Section label="Next Pose">
            <NextPoseCard
              recommendations={recommendations}
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
                <Button
                  size="sm"
                  className="flex-1"
                  onClick={() => void captureAction()}
                  disabled={loading}
                >
                  {loading ? "..." : "캡처"}
                </Button>
                <Button
                  size="sm"
                  variant="outline"
                  onClick={() => void handleReset()}
                  disabled={loading || poses.length === 0}
                >
                  리셋
                </Button>
              </div>
              <HandEyePoseList poses={poses} />
            </div>
          </Section>

          <Section label="Commit">
            <div className="flex flex-col gap-2">
              <p className="text-[11px] text-zinc-500">
                σ TSDF GOOD 안이면 sufficient — hand_eye.npz 에 저장.
              </p>
              <Button
                size="sm"
                variant="secondary"
                onClick={() => void handleCommit()}
                disabled={loading || (!compute && !liveSigma) || computeStale}
              >
                COMMIT (저장)
              </Button>
            </div>
          </Section>
        </>
      )}

      {status && (
        <Section label="Status">
          <p className="text-[11px] text-zinc-500">{status}</p>
        </Section>
      )}
    </PanelShell>
  );
}
