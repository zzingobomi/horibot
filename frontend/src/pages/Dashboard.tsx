/**
 * Dashboard — 앱 착지점 (robot-agnostic). `/` 진입 시 특정 robot 페이지로 리다이렉트
 * 하지 않는다 (ambient default 로봇 개념 제거) — 여기서 전체를 조망하고 사용자가
 * robot/task 를 고른다.
 *
 * 시스템 메트릭 = host_monitor fan-in (`GET /hosts`) — 각 host 가 CPU/mem 을 발행하고
 * bridge 가 payload.host 로 모은 걸 여기서 라이브 표시 (docs/logging.md §7, 영속 X).
 * 발견은 동적 — 발행 중인 host 가 나타나고, 끊기면 offline (age 표시). 노드 배치는
 * 아직 placeholder (liveliness 기반 후속).
 */
import { Link } from "react-router-dom";
import { Bot, Cpu, Network } from "lucide-react";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { useRobots } from "@/hooks/useRobots";
import { useResource } from "@/framework/resource";
import type { HostsResponse } from "@/api/generated/contract";

function ComingSoon() {
  return (
    <Badge variant="outline" className="text-muted-foreground">
      데이터 추후 구현
    </Badge>
  );
}

function MetricBar({ label, pct }: { label: string; pct: number }) {
  const clamped = Math.max(0, Math.min(100, pct));
  const hot = clamped >= 85;
  return (
    <div className="flex items-center gap-2 text-xs">
      <span className="w-8 text-muted-foreground">{label}</span>
      <div className="h-1.5 flex-1 overflow-hidden rounded-full bg-zinc-800">
        <div
          className={hot ? "h-full bg-red-500" : "h-full bg-emerald-500"}
          style={{ width: `${clamped}%` }}
        />
      </div>
      <span className="w-10 text-right font-mono text-zinc-300">
        {clamped.toFixed(0)}%
      </span>
    </div>
  );
}

export function Dashboard() {
  const { robots } = useRobots();
  // host_monitor fan-in — 2s 폴링 라이브 (영속 X). 발행 중인 host 만 동적 등장.
  const { data: hostsData } = useResource<HostsResponse>("/hosts", {
    poll: 2000,
  });
  const hosts = hostsData?.hosts ?? [];

  return (
    <div className="h-full overflow-y-auto p-6">
      <header className="mb-6">
        <h1 className="text-xl font-semibold tracking-tight text-zinc-100">
          Dashboard
        </h1>
        <p className="text-sm text-muted-foreground">
          분산 환경 개요 — 호스트 / 노드 / 로봇 상태
        </p>
      </header>

      <div className="grid grid-cols-1 gap-4 md:grid-cols-2 xl:grid-cols-3">
        {/* 로봇 — 실제 데이터 (robots.yaml SSOT) */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Bot className="h-4 w-4" /> 로봇 ({robots.length})
            </CardTitle>
            <CardDescription>연결된 robot 목록</CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col gap-1.5">
            {robots.length === 0 ? (
              <span className="text-sm text-muted-foreground">
                로봇 없음 (백엔드 연결 확인)
              </span>
            ) : (
              robots.map((r) => (
                <Link
                  key={r.id}
                  to={`/robots/${r.id}`}
                  className="flex items-center justify-between rounded-md border border-zinc-800 px-3 py-1.5 text-sm hover:bg-zinc-800/50"
                >
                  <span className="font-mono">{r.id}</span>
                  <span className="text-xs text-muted-foreground">{r.type}</span>
                </Link>
              ))
            )}
          </CardContent>
        </Card>

        {/* 분산 시스템 메트릭 — host_monitor fan-in (GET /hosts), 2s 라이브 */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Cpu className="h-4 w-4" /> 시스템 메트릭 ({hosts.length})
            </CardTitle>
            <CardDescription>호스트별 CPU / 메모리 (라이브)</CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col gap-2">
            {hosts.length === 0 ? (
              <span className="text-sm text-muted-foreground">
                호스트 텔레메트리 없음 (host_monitor 미가동 / 백엔드 연결 확인)
              </span>
            ) : (
              hosts.map((h) => (
                <div
                  key={h.host}
                  className="rounded-md border border-zinc-800 px-3 py-2"
                >
                  <div className="mb-1.5 flex items-center justify-between">
                    <span className="font-mono text-sm text-zinc-100">
                      {h.host}
                    </span>
                    {h.online ? (
                      <Badge
                        variant="outline"
                        className="border-emerald-800 text-emerald-400"
                      >
                        online
                      </Badge>
                    ) : (
                      <Badge variant="outline" className="text-zinc-500">
                        offline · {h.age_s.toFixed(0)}s
                      </Badge>
                    )}
                  </div>
                  <div className={h.online ? "space-y-1" : "space-y-1 opacity-40"}>
                    <MetricBar label="CPU" pct={h.cpu_percent} />
                    <MetricBar label="MEM" pct={h.mem_percent} />
                  </div>
                </div>
              ))
            )}
          </CardContent>
        </Card>

        {/* 노드 배치 — placeholder (노드↔호스트 presence 집계 미구현) */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Network className="h-4 w-4" /> 노드 배치
            </CardTitle>
            <CardDescription>어떤 노드가 어떤 장비에</CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col items-start gap-2">
            <ComingSoon />
            <span className="text-sm text-muted-foreground">
              모듈이 실제로 어느 호스트에 떠 있는지 (liveliness 기반) 표시 예정.
            </span>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
