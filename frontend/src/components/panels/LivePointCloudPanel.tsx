/**
 * Live PointCloud — RGBD primitive 라이브 point cloud 토글 + 시각 옵션.
 *
 * 책임 (SceneControlsPanel 에서 분리):
 * - Enabled 토글 → SCENE3D_SET_STREAM service (consumers refcount + camera depth stream)
 * - Density (voxel down-sample, mm) → backend Scene3DNode 의 voxel_size 갱신
 * - Point Size → frontend Scene3DLayer 의 pointsMaterial.size 시각 옵션 (UI only)
 *
 * 정책 — Density 자리 1mm / 2mm / 5mm 세 단계 radio (사용자 결정, 2026-06-21):
 * 10mm+ 자리 hand-eye / scan / 로봇 위치 확인 자리 정보량 부족.
 */
import * as SliderPrimitive from "@radix-ui/react-slider";
import { Layers } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { useScene3DStore } from "@/domain/stores/scene3D";
import { PanelShell } from "@/components/shared/PanelShell";
import { Section } from "@/components/shared/Section";
import { ToggleRow } from "@/components/shared/ToggleRow";

const DENSITY_OPTIONS: { label: string; mm: number; hint: string }[] = [
  { label: "Fine",   mm: 1, hint: "1 mm — 최고 품질, 무거움" },
  { label: "Normal", mm: 2, hint: "2 mm — 적당한 성능, 권장" },
  { label: "Fast",   mm: 5, hint: "5 mm — 빠름, 대략적 형상" },
];

export function LivePointCloudPanel(props: IDockviewPanelProps<object>) {
  const enabled = useScene3DStore((s) => s.enabled);
  const voxelSize = useScene3DStore((s) => s.voxelSize);
  const pointSize = useScene3DStore((s) => s.pointSize);
  const setEnabled = useScene3DStore((s) => s.setEnabled);
  const setVoxelSize = useScene3DStore((s) => s.setVoxelSize);
  const setPointSize = useScene3DStore((s) => s.setPointSize);

  const currentMm = Math.round(voxelSize * 1000);

  return (
    <PanelShell
      icon={<Layers className="w-3.5 h-3.5" />}
      title="Live PointCloud"
      panelId={props.api.id}
      api={props.api}
    >
      <Section label="Stream">
        <ToggleRow
          label="Enabled"
          checked={enabled}
          onChange={() => setEnabled(!enabled)}
          accentColor="bg-emerald-400"
        />
      </Section>

      <Section label="Density">
        <div className="flex flex-col gap-1">
          {DENSITY_OPTIONS.map((opt) => {
            const selected = currentMm === opt.mm;
            return (
              <button
                key={opt.mm}
                type="button"
                onClick={() => void setVoxelSize(opt.mm / 1000)}
                className={`flex items-center gap-2 px-2 py-1.5 rounded text-[11px] font-mono transition-colors text-left ${
                  selected
                    ? "bg-zinc-800 text-zinc-100"
                    : "text-zinc-400 hover:bg-zinc-800/60 hover:text-zinc-200"
                }`}
              >
                <span
                  className={`w-2.5 h-2.5 rounded-full border ${
                    selected
                      ? "bg-emerald-400 border-emerald-400"
                      : "border-zinc-600"
                  }`}
                />
                <span className="w-12">{opt.label}</span>
                <span className="text-zinc-500 text-[10px]">{opt.hint}</span>
              </button>
            );
          })}
        </div>
      </Section>

      <Section label="Point Size">
        <div className="flex items-center gap-2 font-mono">
          <SliderPrimitive.Root
            className="relative flex items-center select-none touch-none flex-1 h-4"
            min={1}
            max={8}
            step={1}
            value={[pointSize]}
            onValueChange={(v: number[]) => setPointSize(v[0])}
          >
            <SliderPrimitive.Track className="relative h-1 w-full grow rounded-full bg-zinc-800">
              <SliderPrimitive.Range className="absolute h-full rounded-full bg-emerald-500/40" />
            </SliderPrimitive.Track>
            <SliderPrimitive.Thumb className="block h-3.5 w-3.5 rounded-full border border-emerald-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-emerald-400" />
          </SliderPrimitive.Root>
          <span className="text-[10px] text-zinc-400 tabular-nums w-10 text-right">
            {pointSize} px
          </span>
        </div>
      </Section>
    </PanelShell>
  );
}
