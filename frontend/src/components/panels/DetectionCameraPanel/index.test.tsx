// DetectionCameraPanel — 검출 bbox 오버레이 wire 검증.
// DetectionsUpdate 스트림(store 시딩) → SVG bbox 렌더 (최고 후보 강조),
// stale(팔 이동 후) → 오버레이 숨김.

import { beforeEach, describe, expect, it, vi } from "vitest";
import { render } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { RobotProvider } from "@/components/shared/robotOwnership";
import { useFrameworkStore } from "@/framework/store";
import { DetectionCameraPanel } from "./index";

const ROBOT_ID = "so101_6dof_0";
const WIRE = `stream/detector/${ROBOT_ID}/detections`;
const WIRE_ORIENTED = `stream/detector/${ROBOT_ID}/detections_oriented`;

// robot-scoped 패널 (robotOwnership, capability=rgbd) — unit 에선 RobotProvider 로
// so101 바인딩 재현 (withRobotOwnership 이 실전에서 하는 일).

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
        <Route
          path="/robots/:id"
          element={
            <RobotProvider robotId={ROBOT_ID}>
              <DetectionCameraPanel />
            </RobotProvider>
          }
        />
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
    // mask contour (fuchsia) — best 후보는 반투명 채움 (세그멘테이션 영역 확인)
    const contour = getByTestId("detection-contour");
    expect(contour.getAttribute("stroke")).toBe("#d946ef");
    expect(contour.getAttribute("fill")).toBe("#d946ef");
    expect(contour.getAttribute("fill-opacity")).toBe("0.3");
    // grasp yaw 라벨 = 30°
    const text = getAllByTestId("detection-bbox")[0].querySelector("text")!;
    expect(text.textContent).toContain("∠30°");
  });

  it("멀티 프롬프트 합본 update → prompt 별 best 각각 초록 강조 + 라벨", () => {
    // 스윕 통합 (2026-07-19): 한 관측 update 에 pick/place 후보가 합본으로 온다
    // (backend 가 latest-wins 덮임 방지로 프레임당 1건 발행). 각 물체의 best 가
    // 따로 강조/라벨돼야 "카메라 패널에 픽/플레이스 둘 다" 가 성립.
    useFrameworkStore.setState({
      topicData: {
        [WIRE]: {
          robot_id: ROBOT_ID,
          seq: 1,
          timestamp_unix: Date.now() / 1000,
          prompt: "white cube, blue box",
          image_width: 1280,
          image_height: 720,
          candidates: [
            // score desc — cube best, cube 2등, box best
            {
              prompt: "white cube",
              position: [0.2, 0, 0.05],
              score: 0.9,
              base_z: 0,
              height: 0.05,
              bbox_2d: [100, 200, 300, 400],
            },
            {
              prompt: "white cube",
              position: [0.1, 0.1, 0.05],
              score: 0.7,
              base_z: 0,
              height: 0.05,
              bbox_2d: [500, 100, 600, 250],
            },
            {
              prompt: "blue box",
              position: [0.25, -0.05, 0.04],
              score: 0.6,
              base_z: 0,
              height: 0.04,
              bbox_2d: [800, 300, 1100, 600],
            },
          ],
        },
      },
      serviceData: {},
      bridgeConnected: true,
    });
    const { getAllByTestId } = renderPanel();
    const boxes = getAllByTestId("detection-bbox");
    expect(boxes.length).toBe(3);
    const strokes = boxes.map(
      (g) => g.querySelector("rect")!.getAttribute("stroke"),
    );
    // cube best(0) + box best(2) = 초록, cube 2등(1) = 회색
    expect(strokes[0]).toBe("#34d399");
    expect(strokes[1]).not.toBe("#34d399");
    expect(strokes[2]).toBe("#34d399");
    const labels = boxes
      .map((g) => g.querySelector("text")?.textContent ?? null)
      .filter((t) => t != null);
    expect(labels.length).toBe(2);
    expect(labels[0]).toContain("white cube");
    expect(labels[1]).toContain("blue box");
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
