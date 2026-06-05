import { useService } from "@/framework";
import { useSystemStore } from "@/domain/stores/system";
import { Button } from "@/components/ui/button";
import { ServiceKey } from "@/constants/topics";

export function Settings() {
  const nodes = useSystemStore((s) => s.nodes);
  const logs = useSystemStore((s) => s.logs);
  const cfgSvc = useService(ServiceKey.MOTOR_GET_CONFIG);
  const rebootSvc = useService(ServiceKey.MOTOR_REBOOT);
  const configs = cfgSvc.data?.motors ?? [];

  const busy = cfgSvc.pending || rebootSvc.pending;

  return (
    <div className="flex h-full flex-col gap-4 overflow-y-auto p-4">
      <h1 className="text-lg font-semibold">Settings</h1>

      <section className="rounded-lg border bg-card p-4 flex flex-col gap-3">
        <h2 className="text-sm font-semibold">Node Status</h2>
        <div className="flex flex-col gap-2">
          {["motor_node", "camera_node", "calibration_node"].map((name) => {
            const node = nodes[name];
            const status = node?.status ?? "stopped";
            return (
              <div
                key={name}
                className="flex items-center justify-between rounded-md bg-muted p-2"
              >
                <div className="flex items-center gap-2">
                  <span
                    className={`h-2 w-2 rounded-full ${
                      status === "running"
                        ? "bg-green-500"
                        : status === "error"
                          ? "bg-red-500"
                          : "bg-gray-400"
                    }`}
                  />
                  <span className="text-sm font-mono">{name}</span>
                </div>
                <span className="text-xs text-muted-foreground capitalize">
                  {status}
                </span>
              </div>
            );
          })}
        </div>
      </section>

      <section className="rounded-lg border bg-card p-4 flex flex-col gap-3">
        <div className="flex items-center justify-between">
          <h2 className="text-sm font-semibold">Motor Config</h2>
          <Button
            size="sm"
            variant="outline"
            onClick={() => void cfgSvc.call({})}
            disabled={busy}
          >
            Refresh
          </Button>
        </div>

        <div className="flex flex-col gap-2">
          {configs.length === 0 ? (
            <p className="text-sm text-muted-foreground">설정 없음</p>
          ) : (
            configs.map((cfg) => (
              <div
                key={cfg.id}
                className="flex items-center justify-between rounded-md bg-muted p-2"
              >
                <div>
                  <p className="text-sm font-medium">{cfg.name}</p>
                  <p className="text-xs text-muted-foreground">
                    ID: {cfg.id} · {cfg.model} · home: {cfg.home} · [
                    {cfg.limit.min} ~ {cfg.limit.max}]
                  </p>
                </div>
                <Button
                  size="sm"
                  variant="outline"
                  onClick={() => void rebootSvc.call({ id: cfg.id })}
                  disabled={busy}
                >
                  Reboot
                </Button>
              </div>
            ))
          )}
        </div>

        <Button
          variant="destructive"
          size="sm"
          onClick={() => void rebootSvc.call({})}
          disabled={busy}
        >
          Reboot All
        </Button>
      </section>

      <section className="rounded-lg border bg-card p-4 flex flex-col gap-3">
        <h2 className="text-sm font-semibold">System Log</h2>
        <div className="h-48 overflow-y-auto rounded-md bg-muted p-3 text-xs font-mono space-y-1">
          {logs.length === 0 ? (
            <p className="text-muted-foreground">로그 없음</p>
          ) : (
            [...logs].reverse().map((log, i) => (
              <div key={i} className="flex gap-2">
                <span className="text-muted-foreground shrink-0">
                  {new Date(log.timestamp * 1000).toLocaleTimeString()}
                </span>
                <span
                  className={
                    log.level === "error"
                      ? "text-red-400"
                      : log.level === "warn"
                        ? "text-yellow-400"
                        : "text-foreground"
                  }
                >
                  [{log.node}] {log.message}
                </span>
              </div>
            ))
          )}
        </div>
      </section>
    </div>
  );
}
