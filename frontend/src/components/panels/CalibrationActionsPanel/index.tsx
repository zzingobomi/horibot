/**
 * Calibration actions panel — 기존 Calibration 페이지의 Tabs (Intrinsic / HandEye)
 * 흡수.
 *
 * 기존 CalibrationPanel 은 *조회* (status / matrix) 전용 — 이 panel 은 *수행*
 * (capture / commit / reset). 두 panel 동시 mount 자연스러움.
 */
import { Camera } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { PanelShell } from "@/components/shared/PanelShell";
import { IntrinsicTab } from "@/components/panels/CalibrationActionsPanel/IntrinsicTab";
import { HandEyeTab } from "@/components/panels/CalibrationActionsPanel/HandEyeTab";

export function CalibrationActionsPanel(props: IDockviewPanelProps<object>) {
  return (
    <PanelShell
      icon={<Camera className="w-3.5 h-3.5" />}
      title="Calibration Actions"
      panelId={props.api.id}
      api={props.api}
    >
      <Tabs defaultValue="intrinsic" className="flex flex-col gap-2">
        <TabsList className="w-fit">
          <TabsTrigger value="intrinsic">Intrinsic</TabsTrigger>
          <TabsTrigger value="handeye">Hand-Eye</TabsTrigger>
        </TabsList>

        <TabsContent value="intrinsic" className="m-0">
          <IntrinsicTab />
        </TabsContent>

        <TabsContent value="handeye" className="m-0">
          <HandEyeTab />
        </TabsContent>
      </Tabs>
    </PanelShell>
  );
}
