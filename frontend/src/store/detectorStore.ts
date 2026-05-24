import { create } from "zustand";

export interface Detection {
  class: string;
  bbox: [number, number, number, number]; // [x1, y1, x2, y2]
  conf: number;
}

export interface GroundedResult {
  prompt: string;
  position: [number, number, number]; // base frame (m)
  // 이미지 픽셀 좌표. bbox2d는 정규화된 0~1 또는 절대값 둘 다 가능하게 단순화 — 절대 px.
  bbox2d: { x1: number; y1: number; x2: number; y2: number };
  confidence: number;
  timestamp: number;
}

interface DetectorStore {
  detections: Detection[];
  timestamp: number;
  setDetections: (detections: Detection[], timestamp: number) => void;

  groundedResult: GroundedResult | null;
  setGroundedResult: (r: GroundedResult | null) => void;
}

export const useDetectorStore = create<DetectorStore>((set) => ({
  detections: [],
  timestamp: 0,
  setDetections: (detections, timestamp) => set({ detections, timestamp }),

  groundedResult: null,
  setGroundedResult: (r) => set({ groundedResult: r }),
}));
