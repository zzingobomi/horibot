/**
 * movePreviewStore — MovePreviewPanel(UI) ↔ MovePreviewScenePart(Canvas) 브리지.
 *
 * plan-only 궤적 미리보기 (POC). 패널이 backend `motion_preview/plan` 을 호출해
 * 받은 관절 프레임 시퀀스를 store 에 넣으면, scenePart 가 고스트 로봇으로 재생 +
 * TCP 트레이스선 + 목표 마커를 그린다. 실 로봇(live tcp_state 렌더)은 안 움직임.
 *
 * per-robot 키 — robot A/B 패널이 각자 자기 미리보기 (전역 bool 은 두 번째 robot
 * 에서 오발사). 패널 unmount 시 자기 robot 정리.
 *
 * 세 조각:
 *   - target : 입력창 값 (마커용) — 수치 바꾸면 실시간 갱신 (backend 호출 없음)
 *   - plan   : 프리뷰 버튼 클릭 결과 (프레임/트레이스/feasible). token 이 바뀌면
 *              scenePart 가 프레임 0 부터 재생 (매 클릭 처음부터).
 *   - speed  : 재생 배속 (0.5/1/2) — 프레임 재계산 없이 재생 rate 만 (슬로모로
 *              wrist flip 관찰). 프레임 간격이 50Hz 시간등분이라 배속만 시간축 스케일.
 */
import { create } from "zustand";
import type { PreviewModeValue } from "@/api/generated/contract";

export interface PreviewTargetPose {
  /** base frame, m */
  position: [number, number, number];
  /** roll(x)/pitch(y)/yaw(z), degrees — intrinsic XYZ (backend 규약과 일치) */
  rpyDeg: [number, number, number];
  /** true=목표 자세 지정(마커=축 triad) / false=위치만(마커=점, 자세 자유) */
  useOrientation: boolean;
}

export interface PreviewPlan {
  frames: number[][]; // 관절 프레임 (rad), 50Hz 시간등분
  jointNames: string[];
  tcpTrace: [number, number, number][]; // 프레임별 TCP (base frame, m)
  feasible: boolean;
  failAtSample: number | null;
  mode: PreviewModeValue;
  /** 클릭마다 증가 — scenePart 재생을 프레임 0 부터 리셋하는 트리거 */
  token: number;
}

interface MovePreviewState {
  targets: Record<string, PreviewTargetPose | undefined>;
  plans: Record<string, PreviewPlan | undefined>;
  speeds: Record<string, number | undefined>;
  setTarget: (robotId: string, target: PreviewTargetPose | null) => void;
  setPlan: (robotId: string, plan: PreviewPlan | null) => void;
  setSpeed: (robotId: string, speed: number) => void;
  clear: (robotId: string) => void;
}

export const useMovePreviewStore = create<MovePreviewState>((set) => ({
  targets: {},
  plans: {},
  speeds: {},
  setTarget: (robotId, target) =>
    set((s) => ({
      targets: { ...s.targets, [robotId]: target ?? undefined },
    })),
  setPlan: (robotId, plan) =>
    set((s) => ({
      plans: { ...s.plans, [robotId]: plan ?? undefined },
    })),
  setSpeed: (robotId, speed) =>
    set((s) => ({ speeds: { ...s.speeds, [robotId]: speed } })),
  clear: (robotId) =>
    set((s) => ({
      targets: { ...s.targets, [robotId]: undefined },
      plans: { ...s.plans, [robotId]: undefined },
    })),
}));
