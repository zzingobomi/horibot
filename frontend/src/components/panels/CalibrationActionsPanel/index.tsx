/**
 * Calibration Actions panel — Intrinsic / Rollback 탭.
 *
 * Hand-Eye 는 CalibrationCapturePanel / CalibrationComputePanel 로 분리됨
 * ([docs/calibration_ux_rewrite.md] Q3). 본 panel 은 "가끔 쓰는" 카테고리:
 *   - Intrinsic: omx+D405 시나리오에선 SKIP (factory seed), USB UVC 시에만 사용
 *   - Rollback: 비상 시 (σ 후퇴 / 외부 스크립트로 disk 망친 후)
 *
 * Hand-Eye flow 와 시점 다르고 사용 빈도 작아 한 panel 안 탭으로 합집.
 */
import { Settings } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { PanelShell } from "@/components/shared/PanelShell";
import { IntrinsicTab } from "@/components/panels/CalibrationActionsPanel/IntrinsicTab";
import { RollbackTab } from "@/components/panels/CalibrationActionsPanel/RollbackTab";

export function CalibrationActionsPanel(props: IDockviewPanelProps<object>) {
  return (
    <PanelShell
      icon={<Settings className="w-3.5 h-3.5" />}
      title="Calibration Actions"
      panelId={props.api.id}
      api={props.api}
    >
      <Tabs defaultValue="intrinsic" className="flex flex-col gap-2 p-2">
        <TabsList className="w-fit">
          <TabsTrigger value="intrinsic">Intrinsic</TabsTrigger>
          <TabsTrigger value="rollback">Rollback</TabsTrigger>
        </TabsList>

        <TabsContent value="intrinsic" className="m-0">
          <IntrinsicTab />
        </TabsContent>

        <TabsContent value="rollback" className="m-0">
          <RollbackTab />
        </TabsContent>
      </Tabs>
    </PanelShell>
  );
}
