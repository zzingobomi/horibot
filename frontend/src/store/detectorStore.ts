import { create } from "zustand";

import type { components } from "@/api/generated/types";

// backend `core/transport/messages/detector.py` 의 pydantic 모델에서 자동 생성됨.
// drift 방지 — schema 변경 시 `pnpm gen:types` 로 재생성.
export type Detection = components["schemas"]["YoloDetection"];
export type GroundedResult = components["schemas"]["GroundedDetectionResult"];

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
