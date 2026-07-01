/**
 * ModuleNode — 계약 그래프의 module 노드.
 *
 * services / publishes(streams·events) / subscribes 를 섹션으로. domain 별 색.
 * service 는 owner-attach (caller 엣지 없음, §2) — 노드 안에만 표시.
 * 행 클릭 = 그 wire_key 선택 (page 가 스키마 드릴다운 패널 표시).
 */
import { Handle, Position, type Node, type NodeProps } from "@xyflow/react";
import type { GraphModule } from "../types";

export interface ModuleNodeData {
  module: GraphModule;
  [key: string]: unknown;
}
export type ModuleNodeType = Node<ModuleNodeData, "module">;

const DOMAIN_ACCENT: Record<string, string> = {
  motor: "border-amber-500/60",
  motion: "border-sky-500/60",
  camera: "border-emerald-500/60",
};

function accent(domain: string): string {
  return DOMAIN_ACCENT[domain] ?? "border-zinc-600/60";
}

interface RowsProps {
  title: string;
  items: string[];
  tone: string;
}

function Section({ title, items, tone }: RowsProps) {
  if (items.length === 0) return null;
  return (
    <div className="px-2 py-1">
      <p className="text-[9px] uppercase tracking-wider text-zinc-500">{title}</p>
      {items.map((k) => (
        <div
          key={k}
          className={`truncate font-mono text-[10px] leading-[18px] ${tone}`}
          title={k}
        >
          {k.split("/").pop()}
        </div>
      ))}
    </div>
  );
}

export function ModuleNode({ data }: NodeProps<ModuleNodeType>) {
  const m = data.module;
  return (
    <div
      className={`w-[240px] rounded-md border ${accent(
        m.domain,
      )} bg-zinc-900/95 shadow-lg`}
    >
      <Handle type="target" position={Position.Left} className="!bg-zinc-500" />
      <div className="border-b border-zinc-700/60 px-2 py-1.5">
        <div className="flex items-center justify-between gap-2">
          <span className="truncate text-xs font-semibold text-zinc-100">
            {m.id}
          </span>
          <span className="shrink-0 rounded bg-zinc-800 px-1.5 py-0.5 text-[9px] text-zinc-400">
            {m.domain}
          </span>
        </div>
      </div>
      <Section title="services" items={m.services} tone="text-zinc-300" />
      <Section title="publishes ▸" items={m.publishes} tone="text-sky-300" />
      <Section title="◂ subscribes" items={m.subscribes} tone="text-violet-300" />
      <Handle type="source" position={Position.Right} className="!bg-zinc-500" />
    </div>
  );
}
