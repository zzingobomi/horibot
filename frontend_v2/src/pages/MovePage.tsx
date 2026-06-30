/**
 * MovePage — jog + 3D viewer + robot state. frontend_v2 first cut 의 핵심 page.
 *
 * frontend_v2.md §4 + §11 — CSS grid (dockview 박지 X — Step E+).
 * 3 column: RobotStatePanel | 3D Scene | Jog (J/TCP tab).
 */
import { useState } from "react";
import { useParams } from "react-router-dom";
import { RobotStatePanel } from "@/components/motor/RobotStatePanel";
import { RobotSceneContainer } from "@/components/scene/Container";
import { JogJ } from "@/components/jog/JogJ";
import { JogTcp } from "@/components/jog/JogTcp";
import { DEFAULT_ROBOT_ID } from "@/constants";

type JogTab = "joint" | "tcp";

export function MovePage() {
  const { id } = useParams<{ id: string }>();
  const robotId = id ?? DEFAULT_ROBOT_ID;
  const [tab, setTab] = useState<JogTab>("joint");

  return (
    <div className="h-screen grid grid-cols-[280px_1fr_280px] gap-2 p-2 bg-neutral-950">
      {/* Left — Robot state */}
      <div className="overflow-y-auto bg-zinc-900/40 border border-zinc-800 rounded p-3">
        <div className="text-[10px] uppercase tracking-wide text-zinc-500 mb-2">
          {robotId}
        </div>
        <RobotStatePanel robotId={robotId} />
      </div>

      {/* Center — 3D scene */}
      <div className="bg-zinc-900/40 border border-zinc-800 rounded overflow-hidden">
        <RobotSceneContainer focusId={robotId} />
      </div>

      {/* Right — Jog */}
      <div className="overflow-y-auto bg-zinc-900/40 border border-zinc-800 rounded p-3 flex flex-col gap-3">
        <div className="flex gap-1">
          <button
            onClick={() => setTab("joint")}
            className={`flex-1 h-7 rounded border text-[11px] uppercase tracking-wide ${
              tab === "joint"
                ? "bg-emerald-500/20 border-emerald-500/60 text-emerald-200"
                : "bg-zinc-900 border-zinc-800 text-zinc-500"
            }`}
          >
            joint
          </button>
          <button
            onClick={() => setTab("tcp")}
            className={`flex-1 h-7 rounded border text-[11px] uppercase tracking-wide ${
              tab === "tcp"
                ? "bg-amber-500/20 border-amber-500/60 text-amber-200"
                : "bg-zinc-900 border-zinc-800 text-zinc-500"
            }`}
          >
            tcp
          </button>
        </div>

        {tab === "joint" ? <JogJ robotId={robotId} /> : <JogTcp robotId={robotId} />}
      </div>
    </div>
  );
}
