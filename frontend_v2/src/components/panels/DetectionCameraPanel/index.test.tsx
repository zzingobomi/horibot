// DetectionCameraPanel — 검출 bbox 오버레이 wire 검증.
// DetectionsUpdate 스트림(store 시딩) → SVG bbox 렌더 (최고 후보 강조),
// stale(팔 이동 후) → 오버레이 숨김.

import { beforeEach, describe, expect, it, vi } from "vitest";
import { render } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { useFrameworkStore } from "@/framework/store";
import { DetectionCameraPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";
const WIRE = `stream/detector/${ROBOT_ID}/detections`;
const WIRE_ORIENTED = `stream/detector/${ROBOT_ID}/detections_oriented`;

// task 는 backend 바인딩(GET /tasks)으로 robot 을 정함 — unit 에선 so101 바인딩 mock.
vi.mock("@/hooks/useTasks", () => ({
  useTaskRobotId: () => "so101_6dof_0",
  useTasks: () => ({ tasks: [], loading: false, error: null }),
}));

vi.mock("@/hooks/useRobots", () => ({
  useRobots: () => ({
    robots: [
      {
        id: ROBOT_ID,
        type: "so101_6dof",
        has_camera: true,
        base_pose: { x: 0, y: 0, z: 0, yaw_deg: 0 },
        capabilities: [],
      },
    ],
    loading: false,
    error: null,
  }),
}));

function seed(timestampUnix: number) {
  useFrameworkStore.setState({
    topicData: {
      [WIRE]: {
        robot_id: ROBOT_ID,
        seq: 1,
        timestamp_unix: timestampUnix,
        prompt: "white cube",
        image_width: 1280,
        image_height: 720,
        candidates: [
          {
            prompt: "white cube",
            position: [0.2, 0.0, 0.05],
            score: 0.9,
            base_z: 0,
            height: 0.05,
            bbox_2d: [100, 200, 300, 400],
          },
          {
            prompt: "white cube",
            position: [0.1, 0.1, 0.05],
            score: 0.4,
            base_z: 0,
            height: 0.05,
            bbox_2d: [500, 100, 600, 250],
          },
        ],
      },
    },
    serviceData: {},
    bridgeConnected: true,
  });
}

function seedOriented(timestampUnix: number) {
  useFrameworkStore.setState((s) => ({
    topicData: {
      ...s.topicData, // plain 과 공존 가능 (우선순위 테스트)
      [WIRE_ORIENTED]: {
        robot_id: ROBOT_ID,
        seq: 1,
        timestamp_unix: timestampUnix,
        prompt: "white box",
        image_width: 1280,
        image_height: 720,
        candidates: [
          {
            prompt: "white box",
            position: [0.2, 0.0, 0.05],
            score: 0.9,
            base_z: 0,
            height: 0.05,
            grasp_yaw: Math.PI / 6, // 30°
            footprint: [0.1, 0.05],
            bbox_2d: [100, 200, 300, 400],
            obb_2d: [
              [110, 210],
              [290, 230],
              [280, 390],
              [100, 370],
            ],
            mask_contour: [
              [120, 215],
              [285, 232],
              [278, 385],
              [108, 368],
            ],
          },
        ],
      },
    },
    serviceData: {},
    bridgeConnected: true,
  }));
}

function renderPanel() {
  return render(
    <MemoryRouter initialEntries={[`/robots/${ROBOT_ID}`]}>
      <Routes>
        <Route path="/robots/:id" element={<DetectionCameraPanel />} />
      </Routes>
    </MemoryRouter>,
  );
}

beforeEach(() => {
  useFrameworkStore.setState({ topicData: {}, serviceData: {}, bridgeConnected: true });
});

describe("DetectionCameraPanel", () => {
  it("fresh 검출 → bbox 2개 렌더 + 최고 후보 초록 강조", () => {
    seed(Date.now() / 1000);
    const { getAllByTestId, getByTestId } = renderPanel();
    expect(getByTestId("detection-overlay")).toBeTruthy();
    const boxes = getAllByTestId("detection-bbox");
    expect(boxes.length).toBe(2);
    const firstRect = boxes[0].querySelector("rect")!;
    expect(firstRect.getAttribute("stroke")).toBe("#34d399"); // best = 초록
    expect(firstRect.getAttribute("x")).toBe("100");
    const secondRect = boxes[1].querySelector("rect")!;
    expect(secondRect.getAttribute("stroke")).not.toBe("#34d399");
  });

  it("stale(팔 이동 후) → 오버레이 숨김", () => {
    seed(Date.now() / 1000 - 60); // 60초 전 검출
    const { queryByTestId } = renderPanel();
    expect(queryByTestId("detection-overlay")).toBeNull();
  });

  it("검출 없음 → 오버레이 없음 (카메라 뷰만)", () => {
    const { queryByTestId, getByTestId } = renderPanel();
    expect(getByTestId("camera-view")).toBeTruthy();
    expect(queryByTestId("detection-overlay")).toBeNull();
  });

  it("oriented fresh → obb 회전 사각형 + mask contour + yaw 라벨 렌더", () => {
    seedOriented(Date.now() / 1000);
    const { getByTestId, getAllByTestId } = renderPanel();
    expect(getByTestId("detection-overlay")).toBeTruthy();
    // obb 폴리곤 (호박) — 4 코너
    const obb = getByTestId("detection-obb");
    expect(obb.getAttribute("stroke")).toBe("#f59e0b");
    expect(obb.getAttribute("points")).toBe("110,210 290,230 280,390 100,370");
    // mask contour (하늘) 실루엣
    const contour = getByTestId("detection-contour");
    expect(contour.getAttribute("stroke")).toBe("#38bdf8");
    // grasp yaw 라벨 = 30°
    const text = getAllByTestId("detection-bbox")[0].querySelector("text")!;
    expect(text.textContent).toContain("∠30°");
  });

  it("oriented 가 plain 보다 우선 (둘 다 fresh)", () => {
    // plain(2 후보) + oriented(1 후보, obb 有) 동시 → oriented 렌더 (obb 존재).
    seed(Date.now() / 1000);
    seedOriented(Date.now() / 1000);
    const { getByTestId, getAllByTestId } = renderPanel();
    expect(getAllByTestId("detection-bbox").length).toBe(1); // oriented 후보 1개
    expect(getByTestId("detection-obb")).toBeTruthy();
  });
});
