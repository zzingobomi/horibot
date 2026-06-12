/**
 * Calibration UI state — Hand-Eye capture / compute / commit flow 의 single source.
 *
 * 분리된 panel (CalibrationCameraPanel / HandEyePanel) 들이 같은 state 를 읽고
 * 같은 action 을 호출. 컴포넌트 local useState 였으면 panel 간 sync 가 안 됨
 * (camera 의 overlay preview ↔ hand-eye 의 capture/σ 동기화 필요 등).
 *
 * 라이프사이클: RobotCalibrateMode mount 시 `bootstrap()` 1회 호출, unmount 시
 * `dispose()`. capture/compute panel 들은 mount/unmount 와 무관 — store 가
 * subscribe handle 들을 들고 있음.
 */
import { create } from "zustand";
import { bridge } from "@/api/bridge";
import { ServiceKey, Topic } from "@/constants/topics";
import type {
  CalibThresholds,
  ComputeData,
  HandEyePreview,
  HandeyeRecommendationsState,
  HandeyeObservabilityState,
  HandeyeSaturateState,
  HandEyeSigmaState,
  MultiStartRes,
  NextPoseRecommendation,
  NoCandidatesReason,
  PoseMeta,
  RecommendationFailReq,
} from "@/components/panels/calibration/parts/types";

const PREVIEW_STALE_MS = 1500;

interface CalibrationState {
  // ─── 데이터 ───────────────────────────────────────────────
  preview: HandEyePreview | null;
  previewStale: boolean;
  poses: PoseMeta[];
  liveSigma: HandEyeSigmaState | null;
  compute: ComputeData | null;
  computeStale: boolean;
  // recommendations — CALIB_HANDEYE_RECOMMENDATIONS topic 자동 갱신 (매 capture 마다).
  // Phase 1 (manualModeActive=true) frontend 자체 자리 hide.
  recommendations: NextPoseRecommendation[] | null;
  // 빈 추천 시 *왜* 인지 분리 — NextPoseCard 가 분기별 메시지 표시.
  // null = 아직 publish 안 됨 (Phase 1) 또는 recommendations 채워짐 (분기 N/A).
  noCandidatesReason: NoCandidatesReason | null;
  visited: Set<number>;
  activeIndex: number | null;
  thresholds: CalibThresholds | null;
  // saturate — σ 변화율 추적 결과. Phase 2 표시.
  saturate: HandeyeSaturateState | null;
  // observability — 자세 분포의 기하학적 관측성. verdict 만 사용자 안내.
  observability: HandeyeObservabilityState | null;

  // ─── Phase 1/2 분기 ────────────────────────────────────────
  // manualModeActive=true → Phase 1 (수동 자유 자세, 추천/σ hide).
  // 사용자 [수동 모드 종료] 누르면 → exitManualMode() → multi-start BA → false → Phase 2.
  // [리셋] 누르면 다시 true.
  manualModeActive: boolean;

  // ─── UI 플래그 ─────────────────────────────────────────────
  loading: boolean;
  computing: boolean;
  status: string;

  // ─── 내부: subscribe handle / preview stale timer ─────────
  _unsubscribes: (() => void)[];
  _previewTimer: ReturnType<typeof setTimeout> | null;
  _booted: boolean;

  // ─── 라이프사이클 ─────────────────────────────────────────
  bootstrap: () => void;
  dispose: () => void;

  // ─── actions (panel 들이 호출) ─────────────────────────────
  refreshPoses: () => Promise<void>;
  capture: () => Promise<void>;
  reset: () => Promise<void>;
  compute_: () => Promise<void>;
  commit: () => Promise<{ success: boolean; message: string }>;
  moved: (index: number) => void;
  // Phase 1 → 2 전환. [수동 모드 종료] 버튼.
  // multi-start BA 자동 호출 + manualModeActive=false.
  exitManualMode: () => Promise<MultiStartRes | null>;
  // 사용자 명시 신호 — 추천 자세 fail 기록. 다음 추천 시 제외.
  reportFail: (anchorId: string, category: RecommendationFailReq["category"]) => Promise<void>;
}

export const useCalibrationStore = create<CalibrationState>((set, get) => ({
  preview: null,
  previewStale: false,
  poses: [],
  liveSigma: null,
  compute: null,
  computeStale: false,
  recommendations: null,
  noCandidatesReason: null,
  visited: new Set(),
  activeIndex: null,
  thresholds: null,
  saturate: null,
  observability: null,
  manualModeActive: true,

  loading: false,
  computing: false,
  status: "",

  _unsubscribes: [],
  _previewTimer: null,
  _booted: false,

  bootstrap: () => {
    if (get()._booted) return;
    set({ _booted: true });

    // preview enable + topic 구독
    void bridge.callService(ServiceKey.CALIB_HANDEYE_PREVIEW_ENABLE, {
      enabled: true,
    });
    const unsubPreview = bridge.subscribe(
      Topic.CALIB_HANDEYE_PREVIEW,
      (data) => {
        const prev = get()._previewTimer;
        if (prev) clearTimeout(prev);
        const timer = setTimeout(
          () => set({ previewStale: true }),
          PREVIEW_STALE_MS,
        );
        set({
          preview: data as unknown as HandEyePreview,
          previewStale: false,
          _previewTimer: timer,
        });
      },
    );

    // σ live — capture 마다 backend 자동 BA 결과.
    // computeStale 도 함께 false 로 리셋 — capture action 이 stale=true 박지만
    // 자동 BA 응답이 도착하면 fresh σ 가 박힌 거니 [COMMIT] 활성화 자리.
    const unsubSigma = bridge.subscribe(Topic.CALIB_HANDEYE_SIGMA, (data) => {
      set({
        liveSigma: data as unknown as HandEyeSigmaState,
        computeStale: false,
      });
    });

    // 추천 자세 — capture 마다 backend 자동 publish (Phase 1 자체 자리 hide, Phase 2 show).
    const unsubRecs = bridge.subscribe(
      Topic.CALIB_HANDEYE_RECOMMENDATIONS,
      (data) => {
        const s = data as unknown as HandeyeRecommendationsState;
        set({
          recommendations: s.recommendations ?? [],
          noCandidatesReason: s.no_candidates_reason ?? null,
        });
      },
    );

    // Saturate state — σ 변화율 추적 결과. Phase 2 알림.
    const unsubSaturate = bridge.subscribe(
      Topic.CALIB_HANDEYE_SATURATE,
      (data) => {
        set({ saturate: data as unknown as HandeyeSaturateState });
      },
    );

    // Observability — 매 capture 후 자세 분포 진단. verdict 만 사용자 안내.
    const unsubObservability = bridge.subscribe(
      Topic.CALIB_HANDEYE_OBSERVABILITY,
      (data) => {
        set({ observability: data as unknown as HandeyeObservabilityState });
      },
    );

    // 초기 fetch — pose list + thresholds
    void bridge.callService(ServiceKey.CALIB_HANDEYE_LIST_POSES, {}).then(
      (res) => {
        if (!res.success) return;
        const data = res.data as unknown as { poses: PoseMeta[] };
        set({ poses: data.poses ?? [] });
      },
    );
    void bridge.callService(ServiceKey.CALIB_HANDEYE_THRESHOLDS, {}).then(
      (res) => {
        if (!res.success) return;
        set({ thresholds: res.data as unknown as CalibThresholds });
      },
    );

    set({
      _unsubscribes: [
        unsubPreview,
        unsubSigma,
        unsubRecs,
        unsubSaturate,
        unsubObservability,
      ],
    });
  },

  dispose: () => {
    const { _unsubscribes, _previewTimer } = get();
    for (const u of _unsubscribes) u();
    if (_previewTimer) clearTimeout(_previewTimer);
    void bridge.callService(ServiceKey.CALIB_HANDEYE_PREVIEW_ENABLE, {
      enabled: false,
    });
    set({
      _unsubscribes: [],
      _previewTimer: null,
      _booted: false,
      // state 초기화 — 다음 mount 시 깔끔하게
      preview: null,
      previewStale: false,
      poses: [],
      liveSigma: null,
      compute: null,
      computeStale: false,
      recommendations: null,
      noCandidatesReason: null,
      visited: new Set(),
      activeIndex: null,
      saturate: null,
      manualModeActive: true,
      loading: false,
      computing: false,
      status: "",
    });
  },

  refreshPoses: async () => {
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_LIST_POSES,
      {},
    );
    if (res.success) {
      const data = res.data as unknown as { poses: PoseMeta[] };
      set({ poses: data.poses ?? [] });
    }
  },

  capture: async () => {
    set({ loading: true });
    const res = await bridge.callService(ServiceKey.CALIB_HANDEYE_CAPTURE, {});
    set({ loading: false });
    if (res.success) {
      const data = res.data as { pose_count: number; detected: boolean };
      set({
        status: `✅ 포즈 기록됨 (${data.pose_count}개) — [계산]을 눌러 진척 확인`,
        computeStale: true,
      });
      await get().refreshPoses();
    } else {
      set({ status: `❌ ${res.message}` });
    }
  },

  reset: async () => {
    set({ loading: true });
    const res = await bridge.callService(ServiceKey.CALIB_HANDEYE_RESET, {});
    set({ loading: false });
    if (res.success) {
      set({
        status: "리셋됨 — 자세 잡고 [캡처]부터 시작 (수동 모드)",
        compute: null,
        computeStale: false,
        recommendations: null,
        noCandidatesReason: null,
        visited: new Set(),
        activeIndex: null,
        liveSigma: null,
        saturate: null,
        manualModeActive: true,
      });
      await get().refreshPoses();
    }
  },

  compute_: async () => {
    set({ loading: true, computing: true });
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_COMPUTE,
      {},
      { timeoutMs: 5 * 60 * 1000 },
    );
    set({ loading: false, computing: false });
    if (res.success) {
      const data = res.data as ComputeData;
      set({
        compute: data,
        computeStale: false,
        recommendations: data.recommendations ?? [],
        visited: new Set(),
        activeIndex: null,
        status: "계산 완료. 후보 [이동]→[캡처] 반복, 만족하면 COMMIT.",
      });
    } else {
      set({ compute: null, status: `❌ ${res.message}` });
    }
  },

  commit: async () => {
    set({ loading: true });
    const res = await bridge.callService(ServiceKey.CALIB_HANDEYE_COMMIT, {});
    set({
      loading: false,
      status: res.success ? `✅ ${res.message}` : `❌ ${res.message}`,
    });
    return { success: res.success, message: res.message };
  },

  moved: (index) => {
    set((s) => {
      const next = new Set(s.visited);
      next.add(index);
      return { activeIndex: index, visited: next };
    });
  },

  exitManualMode: async () => {
    // Phase 1 → 2 전환. multi-start BA 자동 호출 (local minimum escape).
    set({ loading: true, status: "Multi-start BA 실행 중..." });
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_MULTI_START,
      { n_starts: 10, mode: "physical_sag" },
      { timeoutMs: 5 * 60 * 1000 },
    );
    set({ loading: false });
    if (res.success) {
      const data = res.data as unknown as MultiStartRes;
      set({
        manualModeActive: false,
        status: `자동 모드 진입 — σ_rot=${data.sigma_rot_deg?.toFixed(2)}° / σ_t=${data.sigma_t_mm?.toFixed(1)}mm (n_converged=${data.n_converged}/${data.n_tried})`,
      });
      return data;
    } else {
      set({ status: `❌ multi-start 실패: ${res.message}` });
      return null;
    }
  },

  reportFail: async (anchorId, category) => {
    const res = await bridge.callService(
      ServiceKey.CALIB_HANDEYE_RECOMMENDATION_FAIL,
      { anchor_id: anchorId, category },
    );
    if (!res.success) {
      set({ status: `❌ fail 신호 실패: ${res.message}` });
    }
    // backend 가 추천 즉시 갱신 publish — subscribe 가 받아 처리
  },
}));
