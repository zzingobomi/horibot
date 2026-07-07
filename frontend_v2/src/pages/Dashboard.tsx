/**
 * Dashboard — 앱 착지점 (robot-agnostic). `/` 진입 시 특정 robot 페이지로 리다이렉트
 * 하지 않는다 (ambient default 로봇 개념 제거) — 여기서 전체를 조망하고 사용자가
 * robot/task 를 고른다.
 *
 * 현재는 **UI 틀만** — 분산 환경(각 호스트 CPU/mem, 노드 배치)은 백엔드 텔레메트리가
 * 아직 없어 placeholder. `GET /system` 은 bridge 호스트(PC) 1대만 노출하므로 여기선
 * 쓰지 않고, 호스트별/노드별 데이터가 생기면 채운다.
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

function ComingSoon() {
  return (
    <Badge variant="outline" className="text-muted-foreground">
      데이터 추후 구현
    </Badge>
  );
}

export function Dashboard() {
  const { robots } = useRobots();

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

        {/* 분산 시스템 메트릭 — placeholder (호스트별 CPU/mem 텔레메트리 미구현) */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Cpu className="h-4 w-4" /> 시스템 메트릭
            </CardTitle>
            <CardDescription>호스트별 CPU / 메모리</CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col items-start gap-2">
            <ComingSoon />
            <span className="text-sm text-muted-foreground">
              PC · 모터 Pi · 카메라 Pi 의 CPU/메모리를 호스트별로 표시 예정.
            </span>
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
