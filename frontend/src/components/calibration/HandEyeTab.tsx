import { useCallback, useEffect, useState } from "react";
import { CameraFeed } from "@/components/camera/CameraFeed";
import { Button } from "@/components/ui/button";
import { CalibJointBar } from "./CalibJointBar";
import type {
  CalibThresholds,
  ComputeData,
  HandEyePreview,
  PoseMeta,
} from "./types";
import { HandEyePoseList } from "./HandEyePoseList";
import { CheckerboardOverlay } from "./CheckerboardOverlay";
import { ComputePreview } from "./HandEyeResults";
import { ServiceKey, Topic } from "@/constants/topics";
import { bridge } from "@/api/bridge";

const PREVIEW_STALE_MS = 1500;

export function HandEyeTab() {
  // Preview
  const [preview, setPreview] = useState<HandEyePreview | null>(null);
  const [previewStale, setPreviewStale] = useState(false);

  // calibration data
  const [poses, setPoses] = useState<PoseMeta[]>([]);
  const [compute, setCompute] = useState<ComputeData | null>(null);
  const [computeStale, setComputeStale] = useState(false);
  const [thresholds, setThresholds] = useState<CalibThresholds | null>(null);

  // utils
  const [loading, setLoading] = useState(false);
  const [computing, setComputing] = useState(false);
  const [status, setStatus] = useState("");

  // checkerboard preview
  useEffect(() => {
    let cancelled = false;
    bridge.callService(ServiceKey.CALIB_HANDEYE_PREVIEW_ENABLE, {
      enabled: true,
    });

    const unsubscribe = bridge.subscribe(
      Topic.CALIB_HANDEYE_PREVIEW,
      (data) => {
        if (cancelled) return;
        setPreview(data as unknown as HandEyePreview);
        setPreviewStale(false);
      }
    );

    return () => {
      cancelled = true;
      unsubscribe();
      bridge.callService(ServiceKey.CALIB_HANDEYE_PREVIEW_ENABLE, {
        enabled: false,
      });
    };
  }, []);

  // preview가 PREVIEW_STALE_MS 동안 갱신 안 되면 stale 표시
  useEffect(() => {
    if (!preview) return;
    const id = window.setTimeout(() => setPreviewStale(true), PREVIEW_STALE_MS);
    return () => window.clearTimeout(id);
  }, [preview]);

  const refreshPoses = useCallback(async () => {
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_LIST_POSES,
      {}
    );
    if (res.success) {
      const data = res.data as { poses: PoseMeta[] };
      setPoses(data.poses ?? []);
    }
  }, []);

  useEffect(() => {
    let cancelled = false;
    bridge.callService(ServiceKey.CALIB_HANDEYE_LIST_POSES, {}).then((res) => {
      if (cancelled || !res.success) return;
      const data = res.data as { poses: PoseMeta[] };
      setPoses(data.poses ?? []);
    });
    bridge
      .callService(ServiceKey.CALIB_HANDEYE_THRESHOLDS, {})
      .then((res) => {
        if (cancelled || !res.success) return;
        setThresholds(res.data as unknown as CalibThresholds);
      });
    return () => {
      cancelled = true;
    };
  }, []);

  const handleCapture = async () => {
    setLoading(true);
    const res = await bridge.callService(ServiceKey.CALIB_HANDEYE_CAPTURE, {});
    setLoading(false);
    if (res.success) {
      const data = res.data as { pose_count: number; detected: boolean };
      setStatus(`✅ 포즈 기록됨 (${data.pose_count}개)`);
      setComputeStale(true);
      await refreshPoses();
    } else {
      setStatus(`❌ ${res.message}`);
    }
  };

  const handleReset = async () => {
    if (!confirm("누적된 모든 포즈를 삭제합니다. 계속할까요?")) return;
    setLoading(true);
    const res = await bridge.callService(ServiceKey.CALIB_HANDEYE_RESET, {});
    setLoading(false);
    if (res.success) {
      setStatus("리셋됨");
      setCompute(null);
      setComputeStale(false);
      await refreshPoses();
    }
  };

  const handleCompute = async () => {
    setLoading(true);
    setComputing(true);
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_COMPUTE,
      {},
      { timeoutMs: 5 * 60 * 1000 }
    );
    setLoading(false);
    setComputing(false);
    if (res.success) {
      setCompute(res.data as unknown as ComputeData);
      setComputeStale(false);
      setStatus("compute 완료. 결과 확인 후 COMMIT 하세요.");
    } else {
      setStatus(`❌ ${res.message}`);
      setCompute(null);
    }
  };

  const handleCommit = async () => {
    setLoading(true);
    const res = await bridge.callService(ServiceKey.CALIB_HANDEYE_COMMIT, {});
    setLoading(false);
    setStatus(res.success ? `✅ ${res.message}` : `❌ ${res.message}`);
  };

  return (
    <div className="flex h-full gap-4 min-h-0">
      {/* 좌측: 카메라 피드 + Joint Control */}
      <div className="flex-1 flex flex-col gap-4 min-h-0">
        <CameraFeed
          className="flex-1 w-full min-h-0"
          overlay={
            <>
              <CalibJointBar />
              <CheckerboardOverlay preview={preview} stale={previewStale} />
            </>
          }
        />
      </div>

      {/* 가운데 컬럼: Capture + Commit */}
      <div className="w-72 shrink-0 flex flex-col gap-3 min-h-0">
        <div className="rounded-lg border bg-card p-4 flex flex-col gap-3 flex-1 min-h-0">
          <h2 className="text-sm font-semibold">Hand-Eye — Capture</h2>
          <p className="text-xs text-muted-foreground">
            다양한 자세에서 캡처 (최소 3개, 권장 10+)
          </p>
          <div className="flex gap-2">
            <Button
              size="sm"
              className="flex-1"
              onClick={handleCapture}
              disabled={loading}
            >
              {loading ? "..." : "캡처"}
            </Button>
            <Button
              size="sm"
              variant="outline"
              onClick={handleReset}
              disabled={loading || poses.length === 0}
            >
              리셋
            </Button>
          </div>

          <div className="flex-1 min-h-0 flex flex-col">
            <HandEyePoseList poses={poses} />
          </div>
        </div>

        <div className="rounded-lg border bg-card p-4 flex flex-col gap-3 shrink-0">
          <h2 className="text-sm font-semibold">Commit</h2>
          <p className="text-xs text-muted-foreground">
            마지막 COMPUTE 결과를 hand_eye.npz에 저장합니다.
          </p>
          <Button
            size="sm"
            variant="secondary"
            onClick={handleCommit}
            disabled={loading || !compute || computeStale}
          >
            COMMIT (저장)
          </Button>
          {status && <p className="text-xs text-muted-foreground">{status}</p>}
        </div>
      </div>

      {/* 우측 컬럼: Compute + Validate */}
      <div className="w-80 shrink-0 flex flex-col gap-3 min-h-0">
        <div className="rounded-lg border bg-card flex flex-col flex-1 min-h-0">
          <div className="p-4 pb-2 flex flex-col gap-3 shrink-0">
            <div className="flex items-center justify-between">
              <h2 className="text-sm font-semibold">Compute</h2>
              {computeStale && compute && (
                <span className="text-[10px] text-amber-500 font-mono">
                  stale
                </span>
              )}
            </div>
            <Button
              size="sm"
              onClick={handleCompute}
              disabled={loading || poses.length < 3}
            >
              {computing ? (
                <span className="inline-flex items-center gap-2">
                  <svg
                    className="animate-spin h-3.5 w-3.5"
                    viewBox="0 0 24 24"
                    fill="none"
                  >
                    <circle
                      cx="12"
                      cy="12"
                      r="10"
                      stroke="currentColor"
                      strokeWidth="4"
                      className="opacity-25"
                    />
                    <path
                      d="M4 12a8 8 0 018-8"
                      stroke="currentColor"
                      strokeWidth="4"
                      strokeLinecap="round"
                    />
                  </svg>
                  계산 중...
                </span>
              ) : (
                "COMPUTE"
              )}
            </Button>
          </div>
          <div className="px-4 pb-4 flex-1 min-h-0 overflow-y-auto relative">
            {compute && thresholds ? (
              <ComputePreview data={compute} thresholds={thresholds} />
            ) : (
              <p className="text-xs text-muted-foreground">
                포즈 캡처 후 COMPUTE를 실행하면 결과 미리보기가 표시됩니다.
              </p>
            )}
            {computing && (
              <div className="absolute inset-0 bg-background/70 backdrop-blur-[1px] flex items-center justify-center pointer-events-none">
                <div className="flex flex-col items-center gap-2 text-xs text-muted-foreground">
                  <svg
                    className="animate-spin h-5 w-5"
                    viewBox="0 0 24 24"
                    fill="none"
                  >
                    <circle
                      cx="12"
                      cy="12"
                      r="10"
                      stroke="currentColor"
                      strokeWidth="4"
                      className="opacity-25"
                    />
                    <path
                      d="M4 12a8 8 0 018-8"
                      stroke="currentColor"
                      strokeWidth="4"
                      strokeLinecap="round"
                    />
                  </svg>
                  Hand-Eye 계산 중...
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
