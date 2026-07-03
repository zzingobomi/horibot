import { Suspense, lazy } from "react";
import { Route, Routes, Navigate } from "react-router-dom";
import { Sidebar } from "@/components/shared/Sidebar";
import { RobotsLayout } from "@/pages/RobotsLayout";
import { RobotModeRedirect } from "@/pages/robotModes/RobotModeRedirect";
import { RobotMoveMode } from "@/pages/robotModes/RobotMoveMode";
import { RobotCalibrateMode } from "@/pages/robotModes/RobotCalibrateMode";
import { RobotScanMode } from "@/pages/robotModes/RobotScanMode";
import { RobotAssetsMode } from "@/pages/robotModes/RobotAssetsMode";
import { useFrameworkBootstrap } from "@/framework";
import { DEFAULT_ROBOT_ID } from "@/constants";

// contract viewer = dev 도구 (§6.1) — lazy import 로 React Flow 번들 code-split
// (control/simulator 경로에 안 섞이게).
const ContractGraphPage = lazy(() =>
  import("@/features/contract-viewer/ContractGraphPage").then((m) => ({
    default: m.ContractGraphPage,
  })),
);

export function App() {
  useFrameworkBootstrap();

  return (
    <div className="flex h-screen overflow-hidden bg-zinc-950">
      <Sidebar />
      <main className="flex-1 overflow-hidden">
        <Routes>
          <Route
            path="/"
            element={<Navigate to={`/robots/${DEFAULT_ROBOT_ID}`} replace />}
          />
          {/* /robots/:id = shared layout (R3F + meta), Outlet 에 mode 컴포넌트.
              mode 전환 시 R3F 는 unmount 안 됨. calibrate 등은 Step E+. */}
          <Route path="/robots/:id" element={<RobotsLayout />}>
            <Route index element={<RobotModeRedirect />} />
            <Route path="move" element={<RobotMoveMode />} />
            <Route path="calibrate" element={<RobotCalibrateMode />} />
            <Route path="scan" element={<RobotScanMode />} />
            <Route path="assets" element={<RobotAssetsMode />} />
          </Route>
          <Route
            path="/contract"
            element={
              <Suspense
                fallback={
                  <div className="flex h-full items-center justify-center text-sm text-zinc-500">
                    contract viewer 로딩 중…
                  </div>
                }
              >
                <ContractGraphPage />
              </Suspense>
            }
          />
        </Routes>
      </main>
    </div>
  );
}
