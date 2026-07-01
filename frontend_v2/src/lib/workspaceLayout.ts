/**
 * dockview mode 별 레이아웃 persist (localStorage).
 *
 * collapse(패널 접기)는 PanelShell 이 도입될 때 함께 — 지금은 패널 chrome 이
 * 없어 collapse 를 구동할 UI 가 없으므로 관련 helper 를 두지 않는다 (반쯤
 * 배선된 dead code 방지, frontend_v2.md §2.3).
 */
function layoutStorageKey(layoutKey: string): string {
  return `workspace.layout.${layoutKey}`;
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
