/**
 * Motion panel — 기존 Motion 페이지의 Tabs 흡수.
 *
 * multi_robot_phase2_frontend.md §2 결정 7 "폼/버튼 → Panel". Motion 페이지가
 * Panel wrapping 이라 페이지 자체는 RobotsPage 의 dockview entry 로 이동.
 *
 * 만져보고 fine tune 자리:
 * - 패널 안의 Tabs vs dockview 자체 tab 분리 (현재는 Tabs UI 유지)
 * - default width / height (현 PANELS 의 motion entry 참조)
 */
import { Cpu } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { PanelShell } from "@/components/canvas/ui/PanelShell";
import { JointPanel } from "@/components/robot/JointPanel";
import { MoveTCPControl } from "@/components/robot/MoveTCPControl";
import { MoveJControl } from "@/components/robot/MoveJControl";
import { MoveLControl } from "@/components/robot/MoveLControl";
import { MoveCControl } from "@/components/robot/MoveCControl";
import { MovePControl } from "@/components/robot/MovePControl";
import { useMotion } from "@/hooks/useMotion";

export function MotionPanel(props: IDockviewPanelProps<object>) {
  const motion = useMotion();

  return (
    <PanelShell
      icon={<Cpu className="w-3.5 h-3.5" />}
      title="Motion"
      panelId={props.api.id}
      api={props.api}
    >
      <Tabs defaultValue="joint" className="flex flex-col gap-2">
        <TabsList className="w-fit">
          <TabsTrigger value="joint">Joint</TabsTrigger>
          <TabsTrigger value="move_j">J</TabsTrigger>
          <TabsTrigger value="move_l">L</TabsTrigger>
          <TabsTrigger value="move_c">C</TabsTrigger>
          <TabsTrigger value="move_p">P</TabsTrigger>
          <TabsTrigger value="move_tcp">TCP</TabsTrigger>
        </TabsList>

        <TabsContent value="joint" className="m-0">
          <JointPanel />
        </TabsContent>

        <TabsContent value="move_j" className="m-0 flex flex-col gap-2">
          <MoveJControl
            trajectoryState={motion.trajectoryState}
            onMoveJ={motion.moveJ}
            onStop={motion.stopMotion}
          />
          {motion.error && <p className="text-xs text-destructive">{motion.error}</p>}
        </TabsContent>

        <TabsContent value="move_l" className="m-0 flex flex-col gap-2">
          <MoveLControl
            tcpPose={motion.tcpPose}
            trajectoryState={motion.trajectoryState}
            onGetTCP={motion.getTCP}
            onMoveL={motion.moveL}
            onStop={motion.stopMotion}
          />
          {motion.error && <p className="text-xs text-destructive">{motion.error}</p>}
        </TabsContent>

        <TabsContent value="move_c" className="m-0 flex flex-col gap-2">
          <MoveCControl
            tcpPose={motion.tcpPose}
            trajectoryState={motion.trajectoryState}
            onGetTCP={motion.getTCP}
            onMoveC={motion.moveC}
            onStop={motion.stopMotion}
          />
          {motion.error && <p className="text-xs text-destructive">{motion.error}</p>}
        </TabsContent>

        <TabsContent value="move_p" className="m-0 flex flex-col gap-2">
          <MovePControl
            tcpPose={motion.tcpPose}
            trajectoryState={motion.trajectoryState}
            onGetTCP={motion.getTCP}
            onMoveP={motion.moveP}
            onStop={motion.stopMotion}
          />
          {motion.error && <p className="text-xs text-destructive">{motion.error}</p>}
        </TabsContent>

        <TabsContent value="move_tcp" className="m-0 flex flex-col gap-2">
          <MoveTCPControl
            tcpPose={motion.tcpPose}
            loading={motion.loading}
            onMoveTCP={motion.moveTCP}
            onGetTCP={motion.getTCP}
          />
          {motion.error && <p className="text-xs text-destructive">{motion.error}</p>}
        </TabsContent>
      </Tabs>
    </PanelShell>
  );
}
