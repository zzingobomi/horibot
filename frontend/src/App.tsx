import { BrowserRouter, Routes, Route } from "react-router-dom";
import { Sidebar } from "@/components/shared/Sidebar";
import { useFrameworkBootstrap } from "@/framework";
import "@/domain/handlers"; // 토픽 비즈니스 등록 — module-top side-effect
import { Dashboard } from "@/pages/Dashboard";
import { Settings } from "@/pages/Settings";
import { RobotsLayout } from "@/pages/RobotsLayout";
import { RobotModeRedirect } from "@/pages/robotModes/RobotModeRedirect";
import { RobotMoveMode } from "@/pages/robotModes/RobotMoveMode";
import { RobotCalibrateMode } from "@/pages/robotModes/RobotCalibrateMode";
import { RobotScanMode } from "@/pages/robotModes/RobotScanMode";
import { WorldPage } from "@/pages/WorldPage";
import { TasksPage } from "@/pages/TasksPage";

function AppContent() {
  useFrameworkBootstrap();

  return (
    <div className="flex h-screen overflow-hidden bg-background">
      <Sidebar />
      <main className="flex-1 overflow-hidden">
        <Routes>
          <Route path="/" element={<Dashboard />} />
          {/* /robots/:id 가 shared layout (R3F + meta), 그 안 Outlet 에 mode
              컴포넌트가 렌더 — mode 전환 시 R3F 는 unmount 안 됨.
              capabilities (robots.yaml) 가 sidebar sub-item / route 활성화 결정. */}
          <Route path="/robots/:id" element={<RobotsLayout />}>
            <Route index element={<RobotModeRedirect />} />
            <Route path="move" element={<RobotMoveMode />} />
            <Route path="calibrate" element={<RobotCalibrateMode />} />
            <Route path="scan" element={<RobotScanMode />} />
          </Route>
          <Route path="/world" element={<WorldPage />} />
          <Route path="/tasks/:name" element={<TasksPage />} />
          <Route path="/settings" element={<Settings />} />
        </Routes>
      </main>
    </div>
  );
}

export default function App() {
  return (
    <BrowserRouter>
      <AppContent />
    </BrowserRouter>
  );
}
