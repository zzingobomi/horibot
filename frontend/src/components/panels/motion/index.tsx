/**
 * Motion panel — Joint / Move J/L/C/P/TCP tabs. 각 sub 가 self-subscribe (useTopic
 * + useService) → props drilling 0, 새 sub 추가 = TabsTrigger + Content 한 줄.
 */
import { Cpu } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { PanelShell } from "@/components/shared/PanelShell";
import { MoveJControl } from "./MoveJ";
import { MoveLControl } from "./MoveL";
import { MoveCControl } from "./MoveC";
import { MovePControl } from "./MoveP";
import { MoveTCPControl } from "./MoveTCP";

const TABS = [
  { value: "move_j", label: "J", body: <MoveJControl /> },
  { value: "move_l", label: "L", body: <MoveLControl /> },
  { value: "move_c", label: "C", body: <MoveCControl /> },
  { value: "move_p", label: "P", body: <MovePControl /> },
  { value: "move_tcp", label: "TCP", body: <MoveTCPControl /> },
];

export function MotionPanel(props: IDockviewPanelProps<object>) {
  return (
    <PanelShell
      icon={<Cpu className="w-3.5 h-3.5" />}
      title="Motion"
      panelId={props.api.id}
      api={props.api}
    >
      <Tabs defaultValue="move_j" className="flex flex-col gap-2 px-3 py-2">
        <TabsList className="w-fit !bg-zinc-900/40 !border !border-zinc-800/60 !rounded !p-0.5 !h-auto">
          {TABS.map((t) => (
            <TabsTrigger
              key={t.value}
              value={t.value}
              className="!text-[10px] !font-mono !uppercase !tracking-wide !text-zinc-500 hover:!text-zinc-200 data-active:!bg-zinc-800/60 data-active:!text-zinc-100 dark:data-active:!bg-zinc-800/60 dark:data-active:!text-zinc-100 dark:data-active:!border-transparent !px-2 !py-1 !rounded-sm !shadow-none"
            >
              {t.label}
            </TabsTrigger>
          ))}
        </TabsList>
        {TABS.map((t) => (
          <TabsContent key={t.value} value={t.value} className="m-0 pt-1">
            {t.body}
          </TabsContent>
        ))}
      </Tabs>
    </PanelShell>
  );
}
