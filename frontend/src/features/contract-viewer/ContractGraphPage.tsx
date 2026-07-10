/**
 * ContractGraphPage — /contract route entry (개발자 도구).
 *
 * backend_v2 의 module 계약(service/stream/event)을 노드+방향엣지 그래프로.
 * GET /contract/graph (unfiltered) → dagre layout → React Flow. 엣지/노드 클릭 →
 * payload 스키마 드릴다운 (Swagger 드릴다운 등가).
 *
 * 이 페이지는 앱 "기능"이 아니라 gen:types 와 동일한 HTTP consumer (§1 비-목적).
 * App.tsx 에서 lazy import — React Flow 번들이 control/simulator 에 안 섞이게.
 */
import "@xyflow/react/dist/style.css";
import { useMemo, useState } from "react";
import {
  Background,
  Controls,
  MiniMap,
  ReactFlow,
  type Edge,
} from "@xyflow/react";
import { useContractGraph } from "./useContractGraph";
import { toReactFlow } from "./toReactFlow";
import { ModuleNode } from "./nodes/ModuleNode";
import type { ContractGraph, KeyInfo, ModelSchema } from "./types";

const nodeTypes = { module: ModuleNode };

export function ContractGraphPage() {
  const { data, loading, error } = useContractGraph();
  const [selectedKey, setSelectedKey] = useState<string | null>(null);

  const flow = useMemo(
    () => (data ? toReactFlow(data) : { nodes: [], edges: [] }),
    [data],
  );

  if (loading && !data) {
    return <Centered>계약 그래프 로딩 중…</Centered>;
  }
  if (error) {
    return (
      <Centered>
        <div className="text-red-400">GET /contract/graph 실패: {error}</div>
        <div className="mt-2 text-xs text-zinc-500">
          backend 를 전 module 로드 host 로 띄웠는지 확인 (예: --host mock).
        </div>
      </Centered>
    );
  }
  if (!data) return <Centered>데이터 없음</Centered>;

  return (
    <div className="relative h-full w-full bg-zinc-950">
      <ReactFlow
        nodes={flow.nodes}
        edges={flow.edges}
        nodeTypes={nodeTypes}
        onEdgeClick={(_e: React.MouseEvent, edge: Edge) => {
          const key = (edge.data as { key?: string } | undefined)?.key;
          setSelectedKey(key ?? null);
        }}
        onPaneClick={() => setSelectedKey(null)}
        fitView
        minZoom={0.2}
        proOptions={{ hideAttribution: true }}
      >
        <Background color="#27272a" gap={20} />
        <Controls className="!bg-zinc-800 !border-zinc-700" />
        <MiniMap
          pannable
          zoomable
          className="!bg-zinc-900"
          maskColor="rgba(0,0,0,0.6)"
        />
      </ReactFlow>

      <Legend counts={data} />
      {selectedKey && (
        <SchemaPanel
          wireKey={selectedKey}
          graph={data}
          onClose={() => setSelectedKey(null)}
        />
      )}
    </div>
  );
}

function Centered({ children }: { children: React.ReactNode }) {
  return (
    <div className="flex h-full w-full flex-col items-center justify-center bg-zinc-950 text-sm text-zinc-400">
      {children}
    </div>
  );
}

function Legend({ counts }: { counts: ContractGraph }) {
  return (
    <div className="absolute left-2 top-2 z-10 rounded-md border border-zinc-800 bg-zinc-900/90 px-3 py-2 text-[11px] text-zinc-400">
      <div className="mb-1 font-semibold text-zinc-200">Contract graph</div>
      <div>modules {counts.modules.length} · edges {counts.edges.length}</div>
      <div className="mt-1 flex gap-3">
        <span className="text-sky-300">▬ stream</span>
        <span className="text-violet-300">╌ event</span>
      </div>
      <div className="mt-1 text-zinc-500">service = owner 속성 (엣지 없음)</div>
    </div>
  );
}

function SchemaPanel({
  wireKey,
  graph,
  onClose,
}: {
  wireKey: string;
  graph: ContractGraph;
  onClose: () => void;
}) {
  const info: KeyInfo | undefined = graph.keys[wireKey];
  return (
    <div className="absolute right-2 top-2 bottom-2 z-10 w-80 overflow-y-auto rounded-md border border-zinc-800 bg-zinc-900/95 p-3 text-xs">
      <div className="flex items-start justify-between gap-2">
        <span className="break-all font-mono text-[11px] text-zinc-200">
          {wireKey}
          {info?.draft && (
            <span className="ml-1.5 rounded bg-amber-500/20 px-1 text-[9px] font-semibold text-amber-400">
              DRAFT
            </span>
          )}
        </span>
        <button
          onClick={onClose}
          className="shrink-0 rounded px-1.5 text-zinc-500 hover:bg-zinc-800 hover:text-zinc-200"
        >
          ✕
        </button>
      </div>
      {!info ? (
        <p className="mt-3 text-zinc-500">계약 정보 없음</p>
      ) : info.category === "service" ? (
        <>
          <ModelBlock title="req" name={info.req} models={graph.models} />
          <ModelBlock title="res" name={info.res} models={graph.models} />
        </>
      ) : (
        <>
          <p className="mt-2 text-zinc-500">{info.category}</p>
          <ModelBlock title="payload" name={info.payload} models={graph.models} />
        </>
      )}
    </div>
  );
}

function ModelBlock({
  title,
  name,
  models,
}: {
  title: string;
  name: string;
  models: Record<string, ModelSchema>;
}) {
  const schema = models[name];
  return (
    <div className="mt-3">
      <p className="text-[10px] uppercase tracking-wider text-zinc-500">
        {title}
      </p>
      <p className="font-mono text-[11px] text-emerald-300">{name}</p>
      {schema ? (
        <div className="mt-1 rounded bg-zinc-950/70 p-2">
          {Object.entries(schema).map(([field, ts]) => (
            <div key={field} className="font-mono text-[10px] leading-5">
              <span className="text-zinc-300">{field}</span>
              <span className="text-zinc-600">: </span>
              <span className="text-amber-300">{ts}</span>
            </div>
          ))}
        </div>
      ) : (
        <p className="mt-1 text-[10px] text-zinc-600">(스키마 없음 — primitive/미참조)</p>
      )}
    </div>
  );
}
