// frontend.md §12.4 — RTL matcher (toBeInTheDocument 등) + cleanup.
import "@testing-library/jest-dom/vitest";
import { afterEach } from "vitest";
import { cleanup } from "@testing-library/react";

afterEach(() => {
  cleanup();
});
