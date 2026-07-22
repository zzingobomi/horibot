/**
 * workcellRoiStore — WorkcellRoiPanel(UI) ↔ WorkcellRoiScenePart(Canvas) 브리지.
 *
 * per-robot 키 (`Record<robotId, ...>`) — robot A/B 패널이 각자 자기 ROI 편집
 * (전역 상태는 두 번째 robot 에서 오발사 — robot ownership 불변식).
 *
 * 두 사본:
 *   - saved : 서버 확정값 (snapshot / WORKCELL_CHANGED / SET 응답). backend
 *             instance.yaml 이 SSOT — 이건 그 캐시일 뿐.
 *   - draft : 편집 중 값 (면 핸들 드래그 + 숫자 입력이 공유). **명시 Save 전엔
 *             서버에 안 감** — dirty = draft ≠ saved 파생.
 * hover/drag 강조(activeFace)도 여기 — 씬의 어느 면을 잡았는지 패널 숫자 필드와
 * 동기 강조 (조작 대상이 3D/2D 양쪽에서 같은 것으로 보이게).
 */
import { create } from "zustand";
import type { WorkcellRoi } from "@/api/generated/contract";
import {
  roiEquals,
  type FaceId,
} from "@/components/panels/WorkcellRoiPanel/dragMath";

interface WorkcellRoiState {
  saved: Record<string, WorkcellRoi | undefined>;
  drafts: Record<string, WorkcellRoi | undefined>;
  /** hover/drag 중인 면 — scenePart 가 쓰고 패널이 필드 강조로 읽는다 */
  activeFace: Record<string, FaceId | undefined>;
  /** 3D 박스 표시 여부 — 편집 데이터(draft)는 유지하고 오버레이만 끈다
   *  (점군/스캔만 깨끗이 보고 싶을 때). per-robot: 미설정=표시(true). */
  visible: Record<string, boolean | undefined>;
  /** 서버 확정값 반영. draft 가 깨끗하면(=saved 와 동일했으면) draft 도 따라간다
   *  — 다른 클라이언트/세션의 저장이 내 편집을 덮지 않되(dirty 보호), 안 만지던
   *  화면은 최신을 보여준다. */
  setSaved: (robotId: string, roi: WorkcellRoi) => void;
  setDraft: (robotId: string, roi: WorkcellRoi) => void;
  revert: (robotId: string) => void;
  setActiveFace: (robotId: string, face: FaceId | undefined) => void;
  setVisible: (robotId: string, visible: boolean) => void;
  clear: (robotId: string) => void;
}

export const useWorkcellRoiStore = create<WorkcellRoiState>((set) => ({
  saved: {},
  drafts: {},
  activeFace: {},
  visible: {},
  setSaved: (robotId, roi) =>
    set((s) => {
      const prevSaved = s.saved[robotId];
      const draft = s.drafts[robotId];
      const clean =
        draft === undefined || (prevSaved !== undefined && roiEquals(draft, prevSaved));
      return {
        saved: { ...s.saved, [robotId]: roi },
        drafts: clean ? { ...s.drafts, [robotId]: roi } : s.drafts,
      };
    }),
  setDraft: (robotId, roi) =>
    set((s) => ({ drafts: { ...s.drafts, [robotId]: roi } })),
  revert: (robotId) =>
    set((s) => ({ drafts: { ...s.drafts, [robotId]: s.saved[robotId] } })),
  setActiveFace: (robotId, face) =>
    set((s) => ({ activeFace: { ...s.activeFace, [robotId]: face } })),
  setVisible: (robotId, visible) =>
    set((s) => ({ visible: { ...s.visible, [robotId]: visible } })),
  clear: (robotId) =>
    set((s) => ({
      saved: { ...s.saved, [robotId]: undefined },
      drafts: { ...s.drafts, [robotId]: undefined },
      activeFace: { ...s.activeFace, [robotId]: undefined },
    })),
}));

/** dirty = draft 가 있고 saved 와 다름 (없던 ROI 를 새로 만든 경우 포함). */
export function isRoiDirty(
  draft: WorkcellRoi | undefined,
  saved: WorkcellRoi | undefined,
): boolean {
  if (!draft) return false;
  if (!saved) return true;
  return !roiEquals(draft, saved);
}
