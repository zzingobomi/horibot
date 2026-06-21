import { useState } from "react";
import { Eye, EyeOff, ChevronDown, ChevronRight } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { useSceneStore, type SceneOptions } from "@/domain/stores/scene";
import { PanelShell } from "@/components/shared/PanelShell";
import { Section } from "@/components/shared/Section";
import { ToggleRow } from "@/components/shared/ToggleRow";

const VISIBILITY_ITEMS: {
  key: keyof SceneOptions;
  label: string;
  color: string;
}[] = [
  { key: "showRobot", label: "Robot", color: "bg-blue-400" },
  { key: "showBaseFrame", label: "Base Frame", color: "bg-white" },
  { key: "showTCPFrame", label: "TCP Frame", color: "bg-yellow-400" },
  { key: "showCameraFrame", label: "Camera Frame", color: "bg-cyan-400" },
  { key: "showGrid", label: "Grid", color: "bg-zinc-400" },
];

export function SceneControlsPanel(props: IDockviewPanelProps<object>) {
  const options = useSceneStore((s) => s.options);
  const linkNames = useSceneStore((s) => s.linkNames);
  const linkVisibility = useSceneStore((s) => s.linkVisibility);
  const toggleOption = useSceneStore((s) => s.toggleOption);
  const toggleLink = useSceneStore((s) => s.toggleLink);
  const toggleAllLinks = useSceneStore((s) => s.toggleAllLinks);

  const [linksExpanded, setLinksExpanded] = useState(false);

  const allVisible =
    linkNames.length > 0 && linkNames.every((n) => linkVisibility[n] !== false);

  return (
    <PanelShell
      icon={<Eye className="w-3.5 h-3.5" />}
      title="Scene Controls"
      panelId={props.api.id}
      api={props.api}
    >
      <Section label="Visibility">
        <div className="space-y-1">
          {VISIBILITY_ITEMS.map(({ key, label, color }) => (
            <ToggleRow
              key={key}
              label={label}
              checked={options[key]}
              onChange={() => toggleOption(key)}
              accentColor={color}
            />
          ))}
        </div>
      </Section>

      <Section label="Robot Links">
        <div className="flex items-center justify-between mb-2">
          <button
            onClick={() => setLinksExpanded((p) => !p)}
            className="flex items-center gap-1 text-[10px] text-zinc-500 hover:text-zinc-300 transition-colors font-mono"
          >
            {linksExpanded ? (
              <ChevronDown className="w-3 h-3" />
            ) : (
              <ChevronRight className="w-3 h-3" />
            )}
            {linksExpanded ? "collapse" : "expand"}
          </button>

          {linkNames.length > 0 && (
            <button
              onClick={toggleAllLinks}
              className="flex items-center gap-1 text-[10px] font-mono text-zinc-500 hover:text-zinc-300 transition-colors"
            >
              {allVisible ? (
                <EyeOff className="w-3 h-3" />
              ) : (
                <Eye className="w-3 h-3" />
              )}
              {allVisible ? "hide all" : "show all"}
            </button>
          )}
        </div>

        {linksExpanded && (
          <div className="space-y-1">
            {linkNames.length === 0 ? (
              <p className="text-[11px] text-zinc-600 font-mono pl-3">
                Loading…
              </p>
            ) : (
              linkNames.map((name) => (
                <ToggleRow
                  key={name}
                  label={name}
                  checked={linkVisibility[name] !== false}
                  onChange={() => toggleLink(name)}
                  accentColor="bg-blue-400"
                />
              ))
            )}
          </div>
        )}
      </Section>
    </PanelShell>
  );
}
