/**
 * capability 어휘 → 사람이 읽는 라벨 + "패널이 요구하는 capability" 판정 helper.
 *
 * registry(패널→requiredCapabilities)와 UI 두 소비자(AutoHideHeader 의 disabled 항목 /
 * withRobotOwnership 의 unsupported empty state)가 공유하는 순수 모듈. registry ↔
 * robotOwnership 순환 import 를 피하려 여기 둔다. 부족 사유 문구는 요구 capability
 * 에서 **파생**(패널마다 hand-author 아님) → requiredCapabilities 와 drift 불가.
 *
 * ⚠️ requiredCapabilities 는 **UI 힌트지 권한의 원천이 아니다.** robot 이 capability
 * 를 가졌다고 반드시 성공하는 건 아니며(예: detector 미실행 / calibration 미로드),
 * 최종 사용 가능 여부는 백엔드가 계속 판정한다. 여기서는 "capability 상 명백히
 * 불가능"한 경우(예: OMX 엔 rgbd 자체가 없음)만 선제 안내해 무의미한 실패를 막는다.
 * [docs/frontend.md]
 */

// capability slug → 사람이 읽는 라벨. 새 capability(force_torque 등) 추가 시 여기만
// 고치면 헤더 tooltip / empty state 문구가 함께 따라온다.
export const CAPABILITY_LABELS: Record<string, string> = {
  rgbd: "RGB-D 카메라",
  gamepad: "게임패드",
  move: "모션",
  calibrate: "캘리브레이션",
};

function labelFor(cap: string): string {
  return CAPABILITY_LABELS[cap] ?? cap;
}

/** required 중 robot 이 갖지 못한 capability 목록 (없으면 빈 배열). */
export function missingCapabilities(
  required: readonly string[] | undefined,
  robotCapabilities: readonly string[] | undefined,
): string[] {
  if (!required || required.length === 0) return [];
  const have = new Set(robotCapabilities ?? []);
  return required.filter((cap) => !have.has(cap));
}

/**
 * 부족 capability 안내 문구. override 가 있으면 그것을 그대로, 없으면 라벨에서
 * 조립 ("RGB-D 카메라 필요" / "RGB-D 카메라, 모션 필요"). override 는 예외적
 * UX 자리에서만 (§frontend.md — 기본은 파생).
 */
export function describeMissing(
  missing: readonly string[],
  override?: string,
): string {
  if (override) return override;
  if (missing.length === 0) return "";
  return `${missing.map(labelFor).join(", ")} 필요`;
}
