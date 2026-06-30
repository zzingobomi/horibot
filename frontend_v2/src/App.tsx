import { Route, Routes, Navigate } from "react-router-dom";
import { MovePage } from "./pages/MovePage";
import { useFrameworkBootstrap } from "./framework";
import { DEFAULT_ROBOT_ID } from "./constants";

export function App() {
  useFrameworkBootstrap();

  return (
    <Routes>
      <Route
        path="/"
        element={<Navigate to={`/robots/${DEFAULT_ROBOT_ID}/move`} replace />}
      />
      <Route path="/robots/:id/move" element={<MovePage />} />
    </Routes>
  );
}
