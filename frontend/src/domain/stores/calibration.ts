/**
 * Calibration UI state — capture-only 시나리오 (online BA / 추천 / σ / observability
 * 전부 폐기, offline Python 스크립트가 분석).
 *
 * 라이프사이클: RobotCalibrateMode mount 시 `bootstrap()` 1회, unmount 시 `dispose()`.
 * panel 들 (CalibrationPanel / CameraPanel) mount/unmount 와 무관 — store 가
 * subscribe handle 들을 들고 있음.
 */
import { create } from "zustand";

import { bridge } from "@/api/bridge";
import { ServiceKey, Topic } from "@/constants/topics";

const PREVIEW_STALE_MS = 1500;

// Scene3DNode 의 SCENE3D_SET_STREAM 자리 acquire/release. 캘 세션과 lifecycle 동기화 —
// 사용자 토글 따로 안 해도 [캘 시작] 자리 depth stream on, [세션 종료]/[리셋] 자리 off.
// 같은 "stream" 토큰 자리라 Scene Controls UI 의 토글 상태와 일치.
async function setDepthStream(robotId: string, enabled: boolean): Promise<void> {
  try {
    await bridge.callService(
      ServiceKey.SCENE3D_SET_STREAM,
      { enabled },
      { robotId },
    );
  } catch (e) {
    console.warn("SCENE3D_SET_STREAM 실패", e);
  }
}

// ─── Wire types ─────────────────────────────────────────────────

export interface PoseMeta {
  pose_index: number;
  tilt_deg: number | null;
}

export interface HandEyePreview {
  timestamp: number;
  detected: boolean;
  tilt_deg: number | null;
  pose_count: number;
  session_active: boolean;
  capture_verdict: "green" | "yellow" | "red";
  capture_reasons: string[];
  corners_2d: [number, number][];
  marker_outlines: [number, number][][];
}

export interface CalibThresholds {
  handeye_pnp_rms_warn_px: number;
  handeye_pnp_rms_reject_px: number;
  capture_similar_joint_deg: number;
  capture_rot_diversity_deg: number;
  capture_trans_diversity_m: number;
  capture_tilt_edge_margin_deg: number;
  tilt_min_deg: number;
  tilt_max_deg: number;
  intrinsic_rms_good_px: number;
  intrinsic_rms_warn_px: number;
  intrinsic_min_captures: number;
  intrinsic_recommended_captures: number;
  intrinsic_grid_coverage_good: number;
}

interface CalibrationState {
  // ─── 데이터 ───────────────────────────────────────────────
  preview: HandEyePreview | null;
  previewStale: boolean;
  // draft run id — null = 사용자 [캘 시작] 안 누름. 캡처 가능 여부 gate.
  hand_eye_run_id: number | null;
  poses: PoseMeta[];
  thresholds: CalibThresholds | null;

  // ─── UI 플래그 ─────────────────────────────────────────────
  loading: boolean;
  status: string;

  // ─── 내부 ──────────────────────────────────────────────────
  _unsubscribes: (() => void)[];
  _previewTimer: ReturnType<typeof setTimeout> | null;
  _booted: boolean;

  // ─── 라이프사이클 ─────────────────────────────────────────
  bootstrap: (robotId: string) => void;
  dispose: () => void;
  // 현재 robot 의 캘 세션 상태 자리 (mode 진입 시 + invalidation 시) 동기화.
  refreshPoses: (robotId: string) => Promise<void>;

  // ─── actions ───────────────────────────────────────────────
  startSession: (robotId: string) => Promise<{ success: boolean; message: string }>;
  capture: (robotId: string) => Promise<void>;
  undoLastCapture: (robotId: string) => Promise<void>;
  reset: (robotId: string) => Promise<void>;
  finalize: (robotId: string) => Promise<{ success: boolean; message: string }>;
}

export const useCalibrationStore = create<CalibrationState>((set, get) => ({
  preview: null,
  previewStale: false,
  hand_eye_run_id: null,
  poses: [],
  thresholds: null,

  loading: false,
  status: "",

  _unsubscribes: [],
  _previewTimer: null,
  _booted: false,

  bootstrap: (robotId: string) => {
    if (get()._booted) return;

    // Preview subscribe — traffic light verdict + ChArUco overlay corners.
    const previewKey = Topic.CALIB_HANDEYE_PREVIEW.replace(
      "{robot_id}",
      robotId,
    );
    const unsubPreview = bridge.subscribe(previewKey, (data: unknown) => {
      const preview = data as HandEyePreview;
      set({ preview, previewStale: false });
      const prev = get()._previewTimer;
      if (prev) clearTimeout(prev);
      const timer = setTimeout(
        () => set({ previewStale: true }),
        PREVIEW_STALE_MS,
      );
      set({ _previewTimer: timer });
    });

    // Preview enable on mount.
    void bridge.callService(
      ServiceKey.CALIB_HANDEYE_PREVIEW_ENABLE,
      { enabled: true },
      { robotId },
    );

    // Thresholds — mount 1회 fetch.
    void bridge
      .callService(ServiceKey.CALIB_HANDEYE_THRESHOLDS, {}, { robotId })
      .then((res) => {
        if (res?.success && res.data) {
          set({ thresholds: res.data as CalibThresholds });
        }
      });

    // 현재 진행 중 draft + 누적 capture 복원. 진행 중이면 depth stream 켜둠
    // (backend 재시작 / 브라우저 reload 경로 자체 자체).
    void get()
      .refreshPoses(robotId)
      .then(() => {
        if (get().hand_eye_run_id != null) {
          void setDepthStream(robotId, true);
        }
      });

    set({
      _unsubscribes: [unsubPreview],
      _booted: true,
    });
  },

  dispose: () => {
    for (const u of get()._unsubscribes) u();
    const timer = get()._previewTimer;
    if (timer) clearTimeout(timer);
    set({
      _unsubscribes: [],
      _previewTimer: null,
      _booted: false,
      preview: null,
      previewStale: false,
      hand_eye_run_id: null,
      poses: [],
      thresholds: null,
      status: "",
    });
  },

  refreshPoses: async (robotId: string) => {
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_LIST_POSES,
      {},
      { robotId },
    );
    if (res?.success && res.data) {
      const data = res.data as unknown as {
        poses: PoseMeta[];
        pose_count: number;
        run_id: number | null;
      };
      set({
        poses: data.poses,
        hand_eye_run_id: data.run_id,
      });
    }
  },

  startSession: async (robotId: string) => {
    set({ loading: true });
    // depth stream 자리 먼저 켬 — 첫 [캡처] 자리 fresh depth_frame 확보.
    await setDepthStream(robotId, true);
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_START,
      {},
      { robotId },
    );
    set({ loading: false });
    if (res?.success && res.data) {
      const data = res.data as { run_id: number; pose_count: number };
      set({
        hand_eye_run_id: data.run_id,
        poses: [],
        status: `✅ 세션 시작 (run_id=${data.run_id})`,
      });
      return { success: true, message: res.message ?? "" };
    }
    // 시작 실패 시 깰깰 acquire 한 depth stream 되돌림.
    await setDepthStream(robotId, false);
    return { success: false, message: res?.message ?? "세션 시작 실패" };
  },

  capture: async (robotId: string) => {
    set({ loading: true });
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_CAPTURE,
      {},
      { robotId },
    );
    set({ loading: false });
    if (res?.success && res.data) {
      const data = res.data as { detected: boolean; pose_count: number };
      set({ status: `✅ 캡처 #${data.pose_count}` });
      await get().refreshPoses(robotId);
    } else {
      set({ status: `❌ ${res?.message ?? "캡처 실패"}` });
    }
  },

  undoLastCapture: async (robotId: string) => {
    set({ loading: true });
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_UNDO_LAST_CAPTURE,
      {},
      { robotId },
    );
    set({ loading: false });
    if (res?.success && res.data) {
      const data = res.data as { deleted: boolean; pose_count: number };
      if (data.deleted) {
        set({ status: "↩ 마지막 capture 삭제" });
        await get().refreshPoses(robotId);
      } else {
        set({ status: "삭제할 capture 없음" });
      }
    } else {
      set({ status: `❌ ${res?.message ?? "되돌리기 실패"}` });
    }
  },

  reset: async (robotId: string) => {
    set({ loading: true });
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_RESET,
      {},
      { robotId },
    );
    set({ loading: false });
    if (res?.success) {
      set({
        hand_eye_run_id: null,
        poses: [],
        status: "↺ 세션 리셋",
      });
      await setDepthStream(robotId, false);
    } else {
      set({ status: `❌ ${res?.message ?? "리셋 실패"}` });
    }
  },

  finalize: async (robotId: string) => {
    set({ loading: true });
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_FINALIZE,
      {},
      { robotId },
    );
    set({ loading: false });
    if (res?.success && res.data) {
      const data = res.data as { run_id: number; pose_count: number };
      set({
        hand_eye_run_id: null,
        poses: [],
        status: (
          `✅ 세션 종료 — run_id=${data.run_id}, ${data.pose_count}장 저장. ` +
          `offline 분석 스크립트 실행 자리.`
        ),
      });
      await setDepthStream(robotId, false);
      return { success: true, message: res.message ?? "" };
    }
    return { success: false, message: res?.message ?? "세션 종료 실패" };
  },
}));
