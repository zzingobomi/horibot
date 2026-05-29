import { BrowserRouter, Routes, Route } from "react-router-dom";
import { Sidebar } from "@/components/common/Sidebar";
import { useBridge } from "@/hooks/useBridge";
import { Dashboard } from "@/pages/Dashboard";
import { Motion } from "@/pages/Motion";
import { Settings } from "@/pages/Settings";
import { Calibration } from "@/pages/Calibration";
import { Workspace3D } from "@/pages/Workspace3D";
import { PickAndPlace } from "@/pages/PickAndPlace";

function AppContent() {
  useBridge();

  return (
    <div className="flex h-screen overflow-hidden bg-background">
      <Sidebar />
      <main className="flex-1 overflow-hidden">
        <Routes>
          <Route path="/" element={<Dashboard />} />
          <Route path="/motion" element={<Motion />} />
          <Route path="/calibration" element={<Calibration />} />
          <Route path="/workspace" element={<Workspace3D />} />
          <Route path="/pick-and-place" element={<PickAndPlace />} />
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
