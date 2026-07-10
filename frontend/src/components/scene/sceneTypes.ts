// Scene 객체/오버레이 공용 props — Scene.tsx 가 각 컴포넌트에 내려주는 컨텍스트.
// (컴포넌트 파일과 분리 — Fast Refresh 컴포넌트-only export 규칙, sceneOptions 와 동일)
import type { RobotInfo } from "@/api/generated/contract";

export interface SceneObjectProps {
  robots: RobotInfo[];
  /** focus robot id — null = 모두 동등 (Tasks/World overview). */
  focusId: string | null;
}
