import { Route, Routes, Navigate } from "react-router-dom";
import { Sidebar } from "@/components/shared/Sidebar";
import { RobotsLayout } from "@/pages/RobotsLayout";
import { RobotModeRedirect } from "@/pages/robotModes/RobotModeRedirect";
import { RobotMoveMode } from "@/pages/robotModes/RobotMoveMode";
import { useFrameworkBootstrap } from "@/framework";
import { DEFAULT_ROBOT_ID } from "@/constants";

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
          </Route>
        </Routes>
      </main>
    </div>
  );
}
