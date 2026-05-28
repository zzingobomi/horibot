const COLLAPSED_KEY = "workspace3d.collapsed";

export const PANEL_HEADER_HEIGHT = 36;

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

export function loadCollapsed(id: string, fallback = true): boolean {
  try {
    const raw = localStorage.getItem(COLLAPSED_KEY);
    if (!raw) return fallback;
    const obj = JSON.parse(raw) as Record<string, boolean>;
    return obj[id] ?? fallback;
  } catch {
    return fallback;
  }
}

export function saveCollapsed(id: string, collapsed: boolean): void {
  try {
    const raw = localStorage.getItem(COLLAPSED_KEY);
    const obj = (raw ? JSON.parse(raw) : {}) as Record<string, boolean>;
    obj[id] = collapsed;
    localStorage.setItem(COLLAPSED_KEY, JSON.stringify(obj));
  } catch {
    // skip
  }
}

export function resetWorkspaceLayout(layoutKey: string): void {
  try {
    localStorage.removeItem(layoutStorageKey(layoutKey));
    localStorage.removeItem(COLLAPSED_KEY);
  } catch {
    // skip
  }
}
