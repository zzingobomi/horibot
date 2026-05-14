const LAYOUT_KEY = "workspace3d.layout";
const COLLAPSED_KEY = "workspace3d.collapsed";

export const PANEL_HEADER_HEIGHT = 36;

export function loadLayout(): unknown | null {
  try {
    const raw = localStorage.getItem(LAYOUT_KEY);
    return raw ? JSON.parse(raw) : null;
  } catch {
    return null;
  }
}

export function saveLayout(data: unknown): void {
  try {
    localStorage.setItem(LAYOUT_KEY, JSON.stringify(data));
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

export function resetWorkspaceLayout(): void {
  try {
    localStorage.removeItem(LAYOUT_KEY);
    localStorage.removeItem(COLLAPSED_KEY);
  } catch {
    // skip
  }
}
