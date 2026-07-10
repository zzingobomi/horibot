// CalibrationPanel — 세션 lifecycle + bundle detail 계약 검증 (unit).
// 실 backend 흐름(capture verdict / offline BA)은 e2e(calibrate.spec.ts) 몫.
// 여기선 ① start_run kind 계약 ② intrinsic finalize ok=false 시 세션 유지
// (in-flight run 을 UI 가 잃던 결함의 회귀망) ③ ok=true 시 세션 종료
// ④ bundle 행 expand 시 실제 수치 렌더.

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render, waitFor } from "@testing-library/react";
import { bridge } from "@/api/bridge";
import { useFrameworkStore, type ServiceEntry } from "@/framework/store";
import { RobotProvider } from "@/components/shared/robotOwnership";
import { CalibrationPanel } from "./index";

const ROBOT_ID = "omx_f_0";

// finalize 응답 시나리오 — 테스트별 세팅.
let finalizeRes: { ok: boolean; message?: string } = { ok: true };

const HAND_EYE_RECORD = {
  id: 2,
  run_id: 2,
  robot_id: "so101_6dof_0",
  created_at: "2026-06-21T03:18:14+00:00",
  is_active: true,
  sigma_rot: 5.21,
  sigma_t: 7.35,
  effective_sigma_rot: 0.82,
  effective_sigma_t: 7.54,
  kind: "hand_eye",
  result_data: {
    R_cam2gripper: [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ],
    t_cam2gripper: [[0.01], [0.02], [0.1]],
    method: "BA(physical_sag_irls)",
  },
};

function respond(key: string): unknown {
  if (key.endsWith("/start_run")) return { run_id: 11 };
  if (key.endsWith("/abort_run")) return { ok: true };
  if (key.endsWith("/finalize_run")) return finalizeRes;
  if (key.endsWith("/capture")) return { accepted: true, reproj_rms_px: null };
  if (key.endsWith("/snapshot_bundle")) {
    return { robot_id: ROBOT_ID, hand_eye: HAND_EYE_RECORD };
  }
  if (key.endsWith("/list_runs")) return { runs: [] };
  return { ok: true };
}

function mockBridge() {
  return vi
    .spyOn(bridge, "callService")
    // @ts-expect-error — 테스트 stub, 응답 shape 는 respond() 가 책임
    .mockImplementation(async (key, _req, opts) => {
      const wk = bridge.serviceCacheKey(key, (opts as { robotId?: string })?.robotId);
      const entry: ServiceEntry = {
        success: true,
        message: "",
        data: respond(String(key)),
        timestamp: Date.now(),
        pending: false,
      };
      useFrameworkStore.getState().setServiceData(wk, entry);
      return entry;
    });
}

function renderPanel() {
  return render(
    <RobotProvider robotId={ROBOT_ID}>
      <CalibrationPanel />
    </RobotProvider>,
  );
}

beforeEach(() => {
  finalizeRes = { ok: true };
  useFrameworkStore.setState({
    topicData: {},
    serviceData: {},
    bridgeConnected: true,
  });
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("CalibrationPanel — 세션 lifecycle", () => {
  it("내부캘 시작 → START_RUN 을 kind=intrinsic + robot_id 로 call, 세션 UI 전환", async () => {
    const spy = mockBridge();
    const { getByTestId, queryByTestId } = renderPanel();

    // idle: 시작 버튼 2개, 진행 버튼 없음 (상태 기반 노출)
    expect(getByTestId("start-intrinsic")).toBeTruthy();
    expect(getByTestId("start-run")).toBeTruthy();
    expect(queryByTestId("capture")).toBeNull();

    await act(async () => {
      fireEvent.click(getByTestId("start-intrinsic"));
    });

    // 계약 자체 assert — robot-agnostic 서비스의 robot_id 는 req 필드 (§2.7)
    expect(spy).toHaveBeenCalledWith(
      "srv/calibration/start_run",
      { robot_id: ROBOT_ID, kind: "intrinsic", algorithm: "charuco_manual" },
      expect.anything(),
    );

    await waitFor(() => expect(getByTestId("capture")).toBeTruthy());
    expect(getByTestId("session-badge").textContent).toContain("내부캘");
    expect(queryByTestId("start-intrinsic")).toBeNull();
  });

  it("finalize ok=false (캡처 부족) → 세션 유지 — in-flight run 을 잃지 않음", async () => {
    finalizeRes = { ok: false, message: "캡처 부족 (2장 < 최소 4장) — 더 캡처 후 재시도" };
    mockBridge();
    const { getByTestId, queryByTestId } = renderPanel();

    await act(async () => {
      fireEvent.click(getByTestId("start-intrinsic"));
    });
    await act(async () => {
      fireEvent.click(getByTestId("finalize"));
    });

    // 회귀망: 세션 살아있음 (캡처 버튼 유지 + 시작 버튼 미노출) + 사유 표시
    expect(getByTestId("capture")).toBeTruthy();
    expect(queryByTestId("start-intrinsic")).toBeNull();
    expect(getByTestId("capture-msg").textContent).toContain("캡처 부족");
  });

  it("중단 → ABORT_RUN call + 세션 즉시 탈출 (0장에서도 — 갇힘 결함 회귀망)", async () => {
    const spy = mockBridge();
    const { getByTestId } = renderPanel();

    await act(async () => {
      fireEvent.click(getByTestId("start-intrinsic"));
    });
    // 캡처 0장 상태에서 중단 — finalize(캡처부족 거부)와 달리 무조건 나가짐
    await act(async () => {
      fireEvent.click(getByTestId("abort"));
    });

    expect(spy).toHaveBeenCalledWith(
      "srv/calibration/abort_run",
      { run_id: 11 },
      expect.anything(),
    );
    await waitFor(() => expect(getByTestId("start-intrinsic")).toBeTruthy());
    expect(getByTestId("capture-msg").textContent).toContain("중단");
  });

  it("finalize ok=true → 세션 종료, 시작 버튼 복귀", async () => {
    mockBridge();
    const { getByTestId } = renderPanel();

    await act(async () => {
      fireEvent.click(getByTestId("start-run"));
    });
    await act(async () => {
      fireEvent.click(getByTestId("finalize"));
    });

    await waitFor(() => expect(getByTestId("start-intrinsic")).toBeTruthy());
    expect(getByTestId("capture-msg").textContent).toContain("종료");
  });
});

describe("CalibrationPanel — bundle detail", () => {
  it("hand_eye 행 클릭 → 실제 수치 expand (mm 변환 + σ dual metric)", async () => {
    mockBridge();
    const { getByTestId, queryByTestId } = renderPanel();

    // snapshot_bundle 응답이 마운트 refresh 로 도착할 때까지
    await waitFor(() =>
      expect(getByTestId("bundle-row-hand_eye").textContent).toContain("σ0.82°"),
    );

    expect(queryByTestId("bundle-detail-hand_eye")).toBeNull();
    await act(async () => {
      fireEvent.click(getByTestId("bundle-row-hand_eye"));
    });

    const detail = getByTestId("bundle-detail-hand_eye");
    // t=[10,20,100]mm — m→mm 변환 계약이 뒤집히면 잡힘
    expect(detail.textContent).toContain("10.0, 20.0, 100.0");
    expect(detail.textContent).toContain("σ_eff");
    expect(detail.textContent).toContain("0.82° / 7.54mm");
    expect(detail.textContent).toContain("run");

    // intrinsic (null) 행은 expand 안 됨
    await act(async () => {
      fireEvent.click(getByTestId("bundle-row-intrinsic"));
    });
    expect(queryByTestId("bundle-detail-intrinsic")).toBeNull();
  });
});
