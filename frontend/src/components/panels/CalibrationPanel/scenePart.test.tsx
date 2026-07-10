// CalibrationScenePart — 보드 3D pose 렌더 계약.
// board_in_cam fresh + tcp 있으면 보드 렌더, 없거나 stale 이면 null.

import { beforeEach, describe, expect, it, vi } from "vitest";
import { render } from "@testing-library/react";
import { RobotProvider } from "@/components/shared/robotOwnership";
import { useMirror, useStream } from "@/framework";
import { CalibrationScenePart } from "./scenePart";

vi.mock("@/framework", () => ({
  useStream: vi.fn(),
  useMirror: vi.fn(),
}));
vi.mock("@/components/scene/shared/RobotFrame", () => ({
  RobotFrame: ({ children }: { children?: React.ReactNode }) => (
    <div data-testid="robot-frame">{children}</div>
  ),
}));
vi.mock("@/components/scene/shared/primitives", () => ({
  Frame: () => null,
}));

const ROBOT_ID = "so101_6dof_0";

const IDENTITY_4X4 = [
  [1, 0, 0, 0],
  [0, 1, 0, 0.2],
  [0, 0, 1, 0.3],
  [0, 0, 0, 1],
];

const TCP = { position: [0.1, 0, 0.2], quaternion: [0, 0, 0, 1] };

function setStreams(opts: {
  board?: number[][] | null;
  boardStale?: boolean;
  tcp?: typeof TCP | null;
}) {
  vi.mocked(useStream).mockImplementation(((topic: string) => {
    if (String(topic).includes("calibration")) {
      return {
        value: opts.board ? { board_in_cam: opts.board } : {},
        stale: opts.boardStale ?? false,
        seq: 1,
        lagMs: 0,
        outOfOrderCount: 0,
      };
    }
    return {
      value: opts.tcp ?? null,
      stale: false,
      seq: 1,
      lagMs: 0,
      outOfOrderCount: 0,
    };
  }) as typeof useStream);
  vi.mocked(useMirror).mockReturnValue({ value: null, isReady: false });
}

function renderPart() {
  return render(
    <RobotProvider robotId={ROBOT_ID}>
      <CalibrationScenePart />
    </RobotProvider>,
  );
}

beforeEach(() => {
  vi.mocked(useStream).mockReset();
  vi.mocked(useMirror).mockReset();
});

describe("CalibrationScenePart", () => {
  it("board_in_cam 없음 → 렌더 안 함", () => {
    setStreams({ board: null, tcp: TCP });
    const { container } = renderPart();
    expect(container.querySelector("mesh")).toBeNull();
  });

  it("preview stale → 숨김 (preview OFF 시 유령 보드 잔존 방지)", () => {
    setStreams({ board: IDENTITY_4X4, boardStale: true, tcp: TCP });
    const { container } = renderPart();
    expect(container.querySelector("mesh")).toBeNull();
  });

  it("fresh board + tcp → 보드 plane 렌더 (RobotFrame 안)", () => {
    setStreams({ board: IDENTITY_4X4, tcp: TCP });
    const { container, getByTestId } = renderPart();
    expect(getByTestId("robot-frame")).toBeTruthy();
    expect(container.querySelector("mesh")).not.toBeNull();
  });
});
