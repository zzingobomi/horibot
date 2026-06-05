import { useBridgeConnected } from "@/framework";
import { useSystemStore } from "@/domain/stores/system";

export function ConnectionStatus() {
  const bridgeConnected = useBridgeConnected();
  const nodes = useSystemStore((s) => s.nodes);

  const nodeList = ["motor_node", "camera_node", "calibration_node"];

  return (
    <div className="flex flex-col gap-2 px-3 py-2">
      <div className="flex items-center gap-2">
        <span
          className={`h-2 w-2 rounded-full ${
            bridgeConnected ? "bg-green-500" : "bg-red-500"
          }`}
        />
        <span className="text-xs text-muted-foreground">
          Bridge {bridgeConnected ? "연결됨" : "끊김"}
        </span>
      </div>

      {nodeList.map((name) => {
        const node = nodes[name];
        const status = node?.status ?? "stopped";
        return (
          <div key={name} className="flex items-center gap-2">
            <span
              className={`h-2 w-2 rounded-full ${
                status === "running"
                  ? "bg-green-500"
                  : status === "error"
                    ? "bg-red-500"
                    : "bg-gray-400"
              }`}
            />
            <span className="text-xs text-muted-foreground truncate">
              {name}
            </span>
          </div>
        );
      })}
    </div>
  );
}
