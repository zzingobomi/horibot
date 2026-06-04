import { BrowserRouter, Routes, Route } from "react-router-dom";
import { Sidebar } from "@/components/common/Sidebar";
import { useBridge } from "@/hooks/useBridge";
import { Dashboard } from "@/pages/Dashboard";
import { Settings } from "@/pages/Settings";
import { RobotsPage } from "@/pages/RobotsPage";
import { WorldPage } from "@/pages/WorldPage";
import { TasksPage } from "@/pages/TasksPage";

function AppContent() {
  useBridge();

  return (
    <div className="flex h-screen overflow-hidden bg-background">
      <Sidebar />
      <main className="flex-1 overflow-hidden">
        <Routes>
          <Route path="/" element={<Dashboard />} />
          {/* multi_robot_phase2_frontend.md §2 — Dashboard / Robots / World /
              Tasks 의 4-페이지 구조. Motion / Calibration / PickAndPlace 는
              Robots / Tasks 의 panel 로 흡수됨 (Slice C). */}
          <Route path="/robots/:id" element={<RobotsPage />} />
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
