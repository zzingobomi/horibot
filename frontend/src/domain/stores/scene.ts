/**
 * Scene UI state — R3F overlay toggle / link visibility / TCP marker.
 *
 * 토픽 / 서비스 자리 아님 — 순수 frontend UI 상태.
 */
import { create } from "zustand";
import type { SceneOptions } from "@/components/scene/Scene";
import type { Vec3 } from "@/types/motion";

export type { SceneOptions };

interface SceneState {
  options: SceneOptions;
  linkNames: string[];
  linkVisibility: Record<string, boolean>;
  tcpPos: Vec3 | null;

  toggleOption: (key: keyof SceneOptions) => void;
  setLinkNames: (names: string[]) => void;
  toggleLink: (name: string) => void;
  toggleAllLinks: () => void;
  setTcpPos: (pos: Vec3 | null) => void;
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
  tcpPos: null,

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

  setTcpPos: (pos) => set({ tcpPos: pos }),
}));
