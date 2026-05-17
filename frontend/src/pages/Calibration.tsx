import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { IntrinsicTab } from "@/components/calibration/IntrinsicTab";
import { HandEyeTab } from "@/components/calibration/HandEyeTab";

export function Calibration() {
  return (
    <div className="flex h-full flex-col gap-4 p-4">
      <Tabs defaultValue="intrinsic" className="flex flex-col flex-1 gap-4">
        <TabsList className="w-fit">
          <TabsTrigger value="intrinsic">Intrinsic</TabsTrigger>
          <TabsTrigger value="handeye">Hand-Eye</TabsTrigger>
        </TabsList>

        <TabsContent value="intrinsic" className="flex-1 m-0">
          <IntrinsicTab />
        </TabsContent>

        <TabsContent value="handeye" className="flex-1 m-0">
          <HandEyeTab />
        </TabsContent>
      </Tabs>
    </div>
  );
}
