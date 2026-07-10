/**
 * LivePointCloudPanel — RGBD 라이브 point cloud 토글 + 시각 옵션 (v1 패널 포팅).
 *
 * 책임 (ScanPanel 에서 분리 — v1 구조 그대로):
 *   - Enabled 토글  → SCENE3D_SET_STREAM (voxel_size 동봉)
 *   - Density(voxel) → backend down-sample 크기. 1/2/5mm 3단계 radio
 *                      (10mm+ 는 hand-eye/scan/로봇 위치 확인에 정보량 부족 —
 *                      사용자 결정 2026-06-21). default 2mm.
 *   - Point Size    → frontend cloud pointsMaterial.size (UI only)
 *   - Camera Frustum → cameraStore.showFrustum (렌더는 Camera 씬 객체 — 패널은 토글만)
 *
 * 3D 렌더는 Canvas 의 Camera 씬 객체(scene/Cameras.tsx) — scanStore/cameraStore 로
 * 결합 (dockview overlay ↔ Canvas). 패널은 카메라를 그리지 않는다 (소유권 모델).
 */
import { useEffect } from "react";
import { Button } from "@/components/ui/button";
import { Slider } from "@/components/ui/slider";
import { useRobotId } from "@/hooks/useRobotId";
import { useMirror, useService, useStream } from "@/framework";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type { CalibrationBundle } from "@/api/generated/contract";
import { useCameraStore } from "@/stores/cameraStore";
import { useScanStore } from "@/stores/scanStore";

const DENSITY_OPTIONS: { label: string; mm: number; hint: string }[] = [
  { label: "Fine", mm: 1, hint: "1 mm — 최고 품질, 무거움" },
  { label: "Normal", mm: 2, hint: "2 mm — 적당한 성능, 권장" },
  { label: "Fast", mm: 5, hint: "5 mm — 빠름, 대략적 형상" },
];

export function LivePointCloudPanel() {
  const robotId = useRobotId();

  const setStream = useService(ServiceKey.SCENE3D_SET_STREAM, robotId);

  // hand-eye 적용 상태 표시 — Camera 씬 객체와 같은 Mirror. cloud 가 identity
  // (캘 미적용) 로 그려지는 상태를 사용자가 즉시 보게 (silent fallback 방지).
  const bundle = useMirror({
    snapshotService: ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE,
    snapshotReq: { robot_id: robotId },
    changeTopic: Topic.CALIBRATION_ACTIVATED,
    robotId,
  });
  const handEye = (bundle.value as CalibrationBundle | null)?.hand_eye ?? null;

  // backend motion FK 캘 적용 상태 — cloud 배치의 나머지 절반 (tcp).
  // hand_eye 가 맞아도 backend 가 무보정 FK 면 cloud 는 어긋난다 (사선/부양).
  // motion 이 무보정으로 떠도 owner 등장 시 자동 수렴 (liveliness Mirror) —
  // 이 배지는 그 수렴 여부를 사용자가 즉시 보게 하는 표면화.
  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId });

  const liveEnabled = useScanStore((s) => s.liveEnabled);
  const setLiveEnabled = useScanStore((s) => s.setLiveEnabled);
  const voxelSize = useScanStore((s) => s.voxelSize);
  const setVoxelSize = useScanStore((s) => s.setVoxelSize);
  const pointSize = useScanStore((s) => s.pointSize);
  const setPointSize = useScanStore((s) => s.setPointSize);
  const showFrustum = useCameraStore((s) => s.showFrustum);
  const setShowFrustum = useCameraStore((s) => s.setShowFrustum);

  // unmount(mode 이탈) 시 stream 정리 — 카메라/PC decode 낭비 X.
  useEffect(() => {
    return () => {
      void setStream.call({ robot_id: robotId, enabled: false });
      setLiveEnabled(false);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const onToggle = async () => {
    const next = !liveEnabled;
    // voxel 동봉 — backend 가 frontend 선택값으로 down-sample (SSOT = 이 패널).
    await setStream.call({
      robot_id: robotId,
      enabled: next,
      voxel_size: voxelSize,
    });
    setLiveEnabled(next);
  };

  const onDensity = async (mm: number) => {
    setVoxelSize(mm / 1000);
    // 켜져 있으면 즉시 반영, 꺼져 있으면 다음 enable 때 동봉.
    if (liveEnabled) {
      await setStream.call({
        robot_id: robotId,
        enabled: true,
        voxel_size: mm / 1000,
      });
    }
  };

  const currentMm = Math.round(voxelSize * 1000);

  return (
    <div
      className="h-full overflow-y-auto p-3 text-[12px]"
      data-testid="live-pc-panel"
    >
      <section className="mb-3">
        <div className="mb-1 flex items-center justify-between">
          <span className="font-mono uppercase text-muted-foreground">stream</span>
          <Button
            size="sm"
            variant={liveEnabled ? "default" : "outline"}
            onClick={onToggle}
            data-testid="live-toggle"
          >
            {liveEnabled ? "ON" : "OFF"}
          </Button>
        </div>
        {/* hand-eye 미적용 = cloud 가 camera frame 그대로 TCP 에 매달림 (사선/부양) */}
        <div className="font-mono text-[10px]" data-testid="handeye-status">
          {!bundle.isReady ? (
            <span className="text-amber-400">hand-eye: 캘 번들 로딩 중…</span>
          ) : handEye ? (
            <span className="text-emerald-500">hand-eye: 적용됨</span>
          ) : (
            <span className="text-red-400">
              hand-eye: 없음 — cloud 위치 부정확 (identity)
            </span>
          )}
        </div>
        {/* backend FK 캘 (joint/link/sag) — 무보정이면 tcp 자체가 틀어짐.
            motion 이 무보정으로 떠도 캘 owner 등장 시 자동 수렴 (liveliness). */}
        <div className="font-mono text-[10px]" data-testid="fk-calib-status">
          {tcp.value == null ? (
            <span className="text-zinc-500">robot FK: 대기 중…</span>
          ) : tcp.value.calibration_stale ? (
            <span className="text-amber-400">
              robot FK: 캘 변경됨 — backend 재시작 필요
            </span>
          ) : tcp.value.calibration_applied ? (
            <span className="text-emerald-500">robot FK: 캘 적용됨</span>
          ) : (
            <span className="text-red-400">
              robot FK: 무보정 — cloud/TCP 부정확 (캘 host 대기)
            </span>
          )}
        </div>
      </section>

      <section className="mb-3">
        <div className="mb-1 font-mono uppercase text-muted-foreground">density</div>
        <div className="flex flex-col gap-1" data-testid="live-pc-density">
          {DENSITY_OPTIONS.map((opt) => {
            const selected = currentMm === opt.mm;
            return (
              <button
                key={opt.mm}
                type="button"
                onClick={() => void onDensity(opt.mm)}
                data-testid={`density-${opt.mm}mm`}
                className={`flex items-center gap-2 rounded px-2 py-1.5 text-left font-mono transition-colors ${
                  selected
                    ? "bg-zinc-800 text-zinc-100"
                    : "text-zinc-400 hover:bg-zinc-800/60 hover:text-zinc-200"
                }`}
              >
                <span
                  className={`h-2.5 w-2.5 rounded-full border ${
                    selected ? "border-emerald-400 bg-emerald-400" : "border-zinc-600"
                  }`}
                />
                <span className="w-12">{opt.label}</span>
                <span className="text-[10px] text-zinc-500">{opt.hint}</span>
              </button>
            );
          })}
        </div>
      </section>

      <section className="mb-3">
        <div className="mb-1 flex items-center justify-between">
          <span className="font-mono uppercase text-muted-foreground">
            camera frustum
          </span>
          <Button
            size="sm"
            variant={showFrustum ? "default" : "outline"}
            onClick={() => setShowFrustum(!showFrustum)}
            data-testid="frustum-toggle"
          >
            {showFrustum ? "ON" : "OFF"}
          </Button>
        </div>
      </section>

      <section>
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          point size
        </div>
        <div className="flex items-center gap-2 font-mono">
          <Slider
            min={1}
            max={8}
            step={0.5}
            value={[pointSize]}
            onValueChange={(v: number[]) => setPointSize(v[0])}
            data-testid="point-size"
          />
          <span className="w-12 text-right text-[10px] tabular-nums text-zinc-400">
            {pointSize} mm
          </span>
        </div>
      </section>
    </div>
  );
}
