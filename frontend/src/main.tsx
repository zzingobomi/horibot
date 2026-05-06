import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./styles/index.css";
import "dockview/dist/styles/dockview.css";
import "./styles/workspace-dockview.css";
import App from "./App.tsx";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
