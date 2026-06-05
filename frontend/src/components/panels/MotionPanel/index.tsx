/**
 * Motion panel — Joint / Move J/L/C/P/TCP tabs. 각 sub 가 self-subscribe (useTopic
 * + useService) → props drilling 0, 새 sub 추가 = TabsTrigger + Content 한 줄.
 */
import { Cpu } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { PanelShell } from "@/components/shared/PanelShell";
import { JointPanel } from "@/components/panels/JointPanel";
import { MoveJControl } from "@/components/panels/MotionPanel/MoveJ";
import { MoveLControl } from "@/components/panels/MotionPanel/MoveL";
import { MoveCControl } from "@/components/panels/MotionPanel/MoveC";
import { MovePControl } from "@/components/panels/MotionPanel/MoveP";
import { MoveTCPControl } from "@/components/panels/MotionPanel/MoveTCP";

const TABS = [
  { value: "joint", label: "Joint", body: <JointPanel /> },
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
      <Tabs defaultValue="joint" className="flex flex-col gap-2">
        <TabsList className="w-fit">
          {TABS.map((t) => (
            <TabsTrigger key={t.value} value={t.value}>
              {t.label}
            </TabsTrigger>
          ))}
        </TabsList>
        {TABS.map((t) => (
          <TabsContent key={t.value} value={t.value} className="m-0">
            {t.body}
          </TabsContent>
        ))}
      </Tabs>
    </PanelShell>
  );
}
