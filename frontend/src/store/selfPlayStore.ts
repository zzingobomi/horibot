import { create } from "zustand";
import {
  defaultSelfPlayState,
  type SelfPlayAttemptResult,
  type SelfPlayState,
} from "@/types/self_play";

const RECENT_LIMIT = 20;

interface SelfPlayStore {
  state: SelfPlayState;
  recentAttempts: SelfPlayAttemptResult[];
  loading: boolean;

  setState: (s: SelfPlayState) => void;
  setLoading: (v: boolean) => void;
  reset: () => void;
}

export const useSelfPlayStore = create<SelfPlayStore>((set, get) => ({
  state: defaultSelfPlayState,
  recentAttempts: [],
  loading: false,

  setState: (s) => {
    // attempt_id 가 바뀌고 last_result 가 있으면 recentAttempts 에 push.
    const prev = get().state;
    const next: { state: SelfPlayState; recentAttempts?: SelfPlayAttemptResult[] } =
      { state: s };
    if (
      s.last_result &&
      s.last_result.attempt_id !== prev.last_result?.attempt_id
    ) {
      const buf = [s.last_result, ...get().recentAttempts].slice(0, RECENT_LIMIT);
      next.recentAttempts = buf;
    }
    set(next);
  },
  setLoading: (v) => set({ loading: v }),
  reset: () =>
    set({
      state: defaultSelfPlayState,
      recentAttempts: [],
      loading: false,
    }),
}));
