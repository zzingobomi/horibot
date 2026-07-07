/**
 * MotionPanel — dockview 등록 패널 (motion). 확장 단위.
 *
 * 패널이 router 의존(useParams)을 여기서 흡수하고, 내부 control (JogJControl /
 * JogTcpControl) 에는 robotId 를 props 로만 내림 → control 은 순수·테스트 용이
 * (frontend_v2.md §2.3, panel = extension unit).
 *
 * 새 motion control 추가 = TabsTrigger + TabsContent 한 줄 (Move* 계층은 backend
 * MoveJ 외 미구현 = Step E+).
 */
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useRobotId } from "@/hooks/useRobotId";
import { JogJControl } from "./JogJControl";
import { JogTcpControl } from "./JogTcpControl";

export function MotionPanel() {
  const robotId = useRobotId();

  return (
    <div className="h-full overflow-y-auto p-3">
      <Tabs defaultValue="joint" className="gap-3">
        <TabsList className="w-full">
          <TabsTrigger value="joint" className="font-mono text-[11px] uppercase">
            joint
          </TabsTrigger>
          <TabsTrigger value="tcp" className="font-mono text-[11px] uppercase">
            tcp
          </TabsTrigger>
        </TabsList>
        <TabsContent value="joint">
          <JogJControl robotId={robotId} />
        </TabsContent>
        <TabsContent value="tcp">
          <JogTcpControl robotId={robotId} />
        </TabsContent>
      </Tabs>
    </div>
  );
}
