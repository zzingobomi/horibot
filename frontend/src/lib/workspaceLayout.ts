/**
 * dockview mode 별 레이아웃 persist (localStorage).
 *
 * collapse(패널 접기)는 PanelShell 이 도입될 때 함께 — 지금은 패널 chrome 이
 * 없어 collapse 를 구동할 UI 가 없으므로 관련 helper 를 두지 않는다 (반쯤
 * 배선된 dead code 방지, frontend.md §2.3).
 */
// v2: 패널의 대상 robot 이 dockview params(params.robotId)에 저장되도록 스키마가
// 바뀜([[robot_ownership_model]]). robotId 없는 옛 v1 레이아웃을 그대로 복원하면
// robot-owned 패널이 Select Robot 빈 상태로 뜨므로, 키를 올려 옛 레이아웃을 폐기
// → 기본 레이아웃이 초기 robotId 를 넣어 재생성. (레이아웃=재생성 가능 자산이라
// 1회 배치 초기화는 허용.)
function layoutStorageKey(layoutKey: string): string {
  return `workspace.layout.v2.${layoutKey}`;
}

export function loadLayout(layoutKey: string): unknown | null {
  try {
    const raw = localStorage.getItem(layoutStorageKey(layoutKey));
    return raw ? JSON.parse(raw) : null;
  } catch {
    return null;
  }
}

export function saveLayout(layoutKey: string, data: unknown): void {
  try {
    localStorage.setItem(layoutStorageKey(layoutKey), JSON.stringify(data));
  } catch {
    // localStorage full / disabled — skip
  }
}

export function resetWorkspaceLayout(layoutKey: string): void {
  try {
    localStorage.removeItem(layoutStorageKey(layoutKey));
  } catch {
    // skip
  }
}
