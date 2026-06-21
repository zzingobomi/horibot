/**
 * Scene UI state — R3F overlay toggle / link visibility.
 *
 * 토픽 / 서비스 자리 아님 — 순수 frontend UI 상태.
 * TCP 위치는 본 store 가 아니라 MOTION_STATE_TCP topic 자리 직접 — backend SSOT.
 */
import { create } from "zustand";
import type { SceneOptions } from "@/components/scene/Scene";

export type { SceneOptions };

interface SceneState {
  options: SceneOptions;
  linkNames: string[];
  linkVisibility: Record<string, boolean>;

  toggleOption: (key: keyof SceneOptions) => void;
  setLinkNames: (names: string[]) => void;
  toggleLink: (name: string) => void;
  toggleAllLinks: () => void;
}

export const useSceneStore = create<SceneState>((set, get) => ({
  options: {
    showRobot: true,
    showBaseFrame: true,
    showTCPFrame: true,
    showCameraFrame: true,
    showGrid: true,
  },
  linkNames: [],
  linkVisibility: {},

  toggleOption: (key) =>
    set((s) => ({ options: { ...s.options, [key]: !s.options[key] } })),

  setLinkNames: (names) =>
    set({
      linkNames: names,
      linkVisibility: Object.fromEntries(names.map((n) => [n, true])),
    }),

  toggleLink: (name) =>
    set((s) => ({
      linkVisibility: { ...s.linkVisibility, [name]: !s.linkVisibility[name] },
    })),

  toggleAllLinks: () => {
    const { linkNames, linkVisibility } = get();
    const allVisible = linkNames.every((n) => linkVisibility[n] !== false);
    set({
      linkVisibility: Object.fromEntries(
        linkNames.map((n) => [n, !allVisible]),
      ),
    });
  },
}));
