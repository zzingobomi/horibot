/**
 * Workspace auto-hide 헤더 ([docs/workspace_autohide_header.md]).
 *
 * 원칙: 이 UI 의 주인공은 3D 씬 — 관리 UI 는 평소 0px, 마우스가 상단으로 향하는
 * 의도의 순간에만 슬라이드 다운. 내용은 `+ 패널 추가`(미배치 목록) 와 `⋯`(레이아웃
 * 초기화) 둘뿐.
 *
 * reveal 규칙 (§3): 상단 REVEAL_THRESHOLD_PX 진입 = reveal, 단 패널 위는 예외.
 * "패널 위" 판정은 좌표 수학 없이 elementFromPoint 한 줄 — workspace 의
 * pointer-events 정책 덕에 빈 영역은 캔버스, 패널 위는 `.dv-resize-container`
 * 가 돌아온다. 메뉴가 열려 있는 동안은 pin (out-timer 정지 — 드롭다운으로 마우스
 * 내릴 때 헤더가 사라지면 메뉴까지 죽으므로 필수).
 */
import { useEffect, useRef, useState } from "react";
import type { DockviewApi } from "dockview";
import { Lock, MoreHorizontal, Plus, RotateCcw } from "lucide-react";
import { describeMissing, missingCapabilities } from "@/lib/capabilities";
import type { PanelSpec } from "./ModeDockview";

// 튜닝값 (§6) — 프로토타입 손끝 조정 대상. 기본값으로 시작.
const REVEAL_THRESHOLD_PX = 16;
const OUT_DELAY_MS = 300;
const IN_DELAY_MS = 120; // 히스테리시스 — 상단을 스쳐 지나갈 때 안 뜨게

interface AutoHideHeaderProps {
  api: DockviewApi | null;
  /** 전체 카탈로그 — 미배치 필터의 모집단 */
  candidates: PanelSpec[];
  /**
   * ambient robot(route :id)의 capability. null = 대상 robot 없음(global/tasks) →
   * 아무것도 disable 하지 않음. capability 부족 항목은 disabled + 사유로 표시.
   */
  ambientCapabilities: string[] | null;
  onAddPanel: (spec: PanelSpec) => void;
  onResetLayout: () => void;
}

export function AutoHideHeader({
  api,
  candidates,
  ambientCapabilities,
  onAddPanel,
  onResetLayout,
}: AutoHideHeaderProps) {
  const [visible, setVisible] = useState(false);
  const [addOpen, setAddOpen] = useState(false);
  const [menuOpen, setMenuOpen] = useState(false);
  // 패널 add/remove 시 미배치 목록 재계산 트리거
  const [, setLayoutVersion] = useState(0);

  const headerRef = useRef<HTMLDivElement | null>(null);
  const inTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const outTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const visibleRef = useRef(visible);
  useEffect(() => {
    visibleRef.current = visible;
  }, [visible]);
  const pinnedRef = useRef(false);
  useEffect(() => {
    pinnedRef.current = addOpen || menuOpen; // 편집중 pin (§2.1)
  }, [addOpen, menuOpen]);

  useEffect(() => {
    if (!api) return;
    const d1 = api.onDidAddPanel(() => setLayoutVersion((v) => v + 1));
    const d2 = api.onDidRemovePanel(() => setLayoutVersion((v) => v + 1));
    return () => {
      d1.dispose();
      d2.dispose();
    };
  }, [api]);

  useEffect(() => {
    const clearIn = () => {
      if (inTimer.current) {
        clearTimeout(inTimer.current);
        inTimer.current = null;
      }
    };
    const clearOut = () => {
      if (outTimer.current) {
        clearTimeout(outTimer.current);
        outTimer.current = null;
      }
    };

    let raf = 0;
    let lastX = 0;
    let lastY = 0;
    const onMove = (e: MouseEvent) => {
      lastX = e.clientX;
      lastY = e.clientY;
      if (raf) return;
      raf = requestAnimationFrame(() => {
        raf = 0;
        const el = document.elementFromPoint(lastX, lastY);
        // elementFromPoint 는 pointer-events:none 을 건너뜀 → 빈 상단이면 캔버스,
        // 패널 위면 .dv-resize-container. closest 유무 하나로 규칙/예외가 갈림 (§3.2).
        const overPanel = !!el?.closest(".dv-resize-container");
        const overHeader = !!(el && headerRef.current?.contains(el));

        if (!visibleRef.current) {
          if (lastY < REVEAL_THRESHOLD_PX && !overPanel) {
            if (!inTimer.current) {
              inTimer.current = setTimeout(() => {
                inTimer.current = null;
                setVisible(true);
              }, IN_DELAY_MS);
            }
          } else {
            clearIn();
          }
        } else if (overHeader || pinnedRef.current) {
          clearOut();
        } else if (!outTimer.current) {
          outTimer.current = setTimeout(() => {
            outTimer.current = null;
            setVisible(false);
            setAddOpen(false);
            setMenuOpen(false);
          }, OUT_DELAY_MS);
        }
      });
    };

    window.addEventListener("mousemove", onMove);
    return () => {
      window.removeEventListener("mousemove", onMove);
      if (raf) cancelAnimationFrame(raf);
      clearIn();
      clearOut();
    };
  }, []);

  // 배치 여부 = component 종류 기준 (id 아님) — mode default 가 같은 종류를 다른
  // id 로 배치해도("camera" 등) "이미 떠 있음"으로 정확히 잡힘.
  const placed = new Set(
    api ? api.panels.map((p) => p.view.contentComponent) : [],
  );
  const missing = candidates.filter((c) => !placed.has(c.component));

  return (
    <>
      {/* 발견성 힌트 — 상단 중앙 얇은 라인 (§2.1). 헤더가 내려오면 숨김 */}
      {!visible && (
        <div className="absolute top-0 left-1/2 -translate-x-1/2 z-30 w-24 h-0.75 rounded-b-full bg-zinc-500/25 pointer-events-none" />
      )}

      <div
        ref={headerRef}
        className={`absolute top-0 left-0 right-0 z-30 transition-transform duration-200 pointer-events-auto ${
          visible ? "translate-y-0" : "-translate-y-full"
        }`}
      >
        <div className="flex items-center justify-end gap-2 px-3 py-1.5 bg-zinc-900/85 backdrop-blur border-b border-zinc-700/60">
          {/* + 패널 추가 — 현재 안 떠 있는 등록 패널 목록 (§2.2) */}
          <div className="relative">
            <button
              onClick={() => {
                setAddOpen((o) => !o);
                setMenuOpen(false);
              }}
              disabled={missing.length === 0}
              title={missing.length === 0 ? "모든 패널이 표시 중" : undefined}
              className="flex items-center gap-1.5 px-2 py-1 rounded bg-zinc-800/80 hover:bg-zinc-700 disabled:opacity-40 disabled:hover:bg-zinc-800/80 border border-zinc-700/60 text-zinc-300 hover:text-zinc-100 text-[10px] font-mono transition-colors"
            >
              <Plus className="w-3 h-3" />
              패널 추가
            </button>
            {addOpen && (
              <div className="absolute right-0 top-full mt-1 min-w-45 rounded border border-zinc-700/60 bg-zinc-900/95 backdrop-blur py-1 shadow-lg">
                {missing.map((m) => {
                  // ambient robot 이 있을 때만 capability 판정 (null=대상 robot 없음).
                  const unmet = ambientCapabilities
                    ? missingCapabilities(m.requiredCapabilities, ambientCapabilities)
                    : [];
                  const disabled = unmet.length > 0;
                  const reason = disabled
                    ? describeMissing(unmet, m.unavailableReason)
                    : undefined;
                  return (
                    <button
                      key={m.id}
                      disabled={disabled}
                      title={reason}
                      onClick={() => {
                        if (disabled) return;
                        onAddPanel(m);
                        setAddOpen(false);
                      }}
                      data-testid={
                        disabled ? "add-panel-disabled" : "add-panel-item"
                      }
                      className={`flex w-full items-center justify-between gap-2 px-3 py-1.5 text-left text-[10px] font-mono ${
                        disabled
                          ? "cursor-not-allowed text-zinc-600"
                          : "text-zinc-300 hover:bg-zinc-700/60 hover:text-zinc-100"
                      }`}
                    >
                      <span className="flex items-center gap-1.5">
                        {disabled && <Lock className="h-2.5 w-2.5" />}
                        {m.title}
                      </span>
                      {reason && (
                        <span className="text-[9px] text-zinc-500">{reason}</span>
                      )}
                    </button>
                  );
                })}
              </div>
            )}
          </div>

          {/* ⋯ 메뉴 — 레이아웃 초기화는 비상용 강등 (§2.2) */}
          <div className="relative">
            <button
              onClick={() => {
                setMenuOpen((o) => !o);
                setAddOpen(false);
              }}
              className="flex items-center px-2 py-1 rounded bg-zinc-800/80 hover:bg-zinc-700 border border-zinc-700/60 text-zinc-400 hover:text-zinc-100 transition-colors"
            >
              <MoreHorizontal className="w-3.5 h-3.5" />
            </button>
            {menuOpen && (
              <div className="absolute right-0 top-full mt-1 min-w-40 rounded border border-zinc-700/60 bg-zinc-900/95 backdrop-blur py-1 shadow-lg">
                <button
                  onClick={() => {
                    setMenuOpen(false);
                    onResetLayout();
                  }}
                  className="flex items-center gap-1.5 w-full text-left px-3 py-1.5 text-[10px] font-mono text-zinc-300 hover:bg-zinc-700/60 hover:text-zinc-100"
                >
                  <RotateCcw className="w-3 h-3" />
                  레이아웃 초기화
                </button>
              </div>
            )}
          </div>
        </div>
      </div>
    </>
  );
}
