/**
 * MovePreviewPanel — plan-only 궤적 미리보기 UI (POC).
 *
 * TCP pose(위치 m + RPY 도)를 입력하면 씬에 목표 마커가 실시간으로 뜨고, 모드
 * 버튼을 누르면 backend `motion_preview/plan` 이 "현재 자세 → 그 pose 로
 * MoveL/MoveJ(pose) 하면" 의 관절 프레임을 계산해 준다. scenePart 가 고스트로 재생
 * (실 로봇은 안 움직임). 두 모드는 같은 목표라도 TCP 경로가 다름 —
 * MoveL=직선 / MoveJ(pose)=곡선 (트레이스선으로 한눈에).
 *
 * 시작 자세 = live Motion.TCP_STATE.joints (backend preview 모듈은 모터 상태를
 * 구독 안 하므로 프론트가 실어 보냄). robot-agnostic 서비스 — req 에 robot_id.
 */
import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { Button } from "@/components/ui/button";
import { useRobotId } from "@/hooks/useRobotId";
import { useService, useStream } from "@/framework";
import { ServiceKey, Topic, PreviewMode } from "@/api/generated/contract";
import type { PreviewModeValue, TcpState } from "@/api/generated/contract";
import { useMovePreviewStore } from "@/stores/movePreviewStore";

const round3 = (v: number) => Math.round(v * 1000) / 1000;
const round1 = (v: number) => Math.round(v * 10) / 10;

// 현재 tcp_state → pose 입력값. rpy 는 현재 orientation 유지(intrinsic XYZ, backend
// 규약과 동일) — 기본 target 이 "현재 자세로 z 살짝 이동" 이라 MoveL 로 도달 가능.
function poseFromTcp(tcp: TcpState, offsetZ: number): PoseInput {
  const [px, py, pz] = tcp.position;
  const [qx, qy, qz, qw] = tcp.quaternion;
  const e = new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(qx, qy, qz, qw),
    "XYZ",
  );
  return {
    x: round3(px),
    y: round3(py),
    z: round3(pz + offsetZ),
    roll: round1(THREE.MathUtils.radToDeg(e.x)),
    pitch: round1(THREE.MathUtils.radToDeg(e.y)),
    yaw: round1(THREE.MathUtils.radToDeg(e.z)),
  };
}

const MODE_LABEL: Record<PreviewModeValue, string> = {
  [PreviewMode.MOVE_L]: "MoveL",
  [PreviewMode.MOVE_J_POSE]: "MoveJ(pose)",
};

const SPEEDS = [0.5, 1, 2] as const;

interface PoseInput {
  x: number;
  y: number;
  z: number;
  roll: number;
  pitch: number;
  yaw: number;
}

export function MovePreviewPanel() {
  const robotId = useRobotId();
  const planSvc = useService(ServiceKey.MOTIONPREVIEW_PLAN, robotId);
  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId });
  const setTarget = useMovePreviewStore((s) => s.setTarget);
  const setPlan = useMovePreviewStore((s) => s.setPlan);
  const setSpeed = useMovePreviewStore((s) => s.setSpeed);
  const speed = useMovePreviewStore((s) => s.speeds[robotId]) ?? 1;
  // 현재 재생 중인 프리뷰의 모드 — 두 버튼 중 어느 것이 활성인지 하이라이트용
  // (변형 variant 차이로 인한 "MoveL 눌렀나 MoveJ 눌렀나" 착각 제거).
  const activeMode = useMovePreviewStore((s) => s.plans[robotId])?.mode ?? null;

  const [pose, setPose] = useState<PoseInput>({
    x: 0,
    y: 0,
    z: 0,
    roll: 0,
    pitch: 0,
    yaw: 0,
  });
  const [seeded, setSeeded] = useState(false);
  const [busy, setBusy] = useState(false);
  const [status, setStatus] = useState("");
  // 자세 축 (motion PoseTarget.quaternion None/set): true=목표 자세 지정(MoveL slerp
  // / MoveJ 도달), false=위치만(자세 자유, 도달성↑). 도달 불가 대부분이 자세 제약이라
  // 이 토글이 "위치는 되나 자세가 blocker" 를 실제로 확인하는 축.
  const [useOrientation, setUseOrientation] = useState(true);
  const tokenRef = useRef(0);

  // 최초 tcp 도착 시 "현재 자세 + z 3cm" 로 seed — 현재 orientation 유지라 기본
  // target 이 MoveL 로 도달 가능 (바로 프리뷰 눌러도 동작하는 출발점).
  useEffect(() => {
    if (seeded) return;
    const v = tcp.value;
    if (!v) return;
    /* eslint-disable react-hooks/set-state-in-effect */
    setPose(poseFromTcp(v, 0.03));
    setSeeded(true);
    /* eslint-enable react-hooks/set-state-in-effect */
  }, [tcp.value, seeded]);

  // 입력 → 마커 실시간 동기 (backend 호출 없이 씬 마커만 이동). useOrientation 이
  // 마커 모양(축 triad vs 점)도 결정.
  useEffect(() => {
    setTarget(robotId, {
      position: [pose.x, pose.y, pose.z],
      rpyDeg: [pose.roll, pose.pitch, pose.yaw],
      useOrientation,
    });
  }, [robotId, pose, useOrientation, setTarget]);

  // 패널 unmount / robot 스위칭 시 미리보기 정리 (남은 고스트 = 주인 없는 표시).
  useEffect(() => () => useMovePreviewStore.getState().clear(robotId), [robotId]);

  const setField = (k: keyof PoseInput, raw: string) => {
    const v = Number.parseFloat(raw);
    setPose((cur) => ({ ...cur, [k]: Number.isFinite(v) ? v : 0 }));
  };

  const seedFromCurrent = () => {
    const v = tcp.value;
    if (!v) return;
    setPose(poseFromTcp(v, 0)); // 현재 TCP 자세 그대로 (offset 없음)
  };

  const runPreview = async (mode: PreviewModeValue) => {
    const joints = tcp.value?.joints;
    if (!joints) {
      setStatus("joint state 대기 중… (motion 미연결?)");
      return;
    }
    setBusy(true);
    setStatus(`계획 중… (${MODE_LABEL[mode]})`);
    try {
      // 궤적을 실시간(50Hz)으로 수집 → 이동 duration 만큼 걸릴 수 있어 기본 5s 로는
      // 부족 (긴 이동은 timeout → reject). 넉넉히 (backend PLAN timeout 도 60s).
      const res = await planSvc.call(
        {
          robot_id: robotId,
          start_joints: joints,
          target: {
            position: [pose.x, pose.y, pose.z],
            rpy_deg: [pose.roll, pose.pitch, pose.yaw],
          },
          mode,
          use_orientation: useOrientation,
        },
        { timeoutMs: 60000 },
      );
      const d = res.data;
      if (!res.success || !d) {
        setStatus(`실패: ${res.message}`);
        setPlan(robotId, null);
        return;
      }
      tokenRef.current += 1;
      setPlan(robotId, {
        frames: d.frames,
        jointNames: d.joint_names,
        tcpTrace: d.tcp_trace,
        feasible: d.feasible,
        failAtSample: d.fail_at_sample ?? null,
        mode,
        token: tokenRef.current,
      });
      setStatus(
        d.feasible
          ? `재생: ${MODE_LABEL[mode]} — ${d.frames.length} 프레임`
          : `⚠ 도달 불가 — ${d.message}`,
      );
    } catch (e) {
      // 실패해도 버튼이 잠기지 않게 (finally 가 busy 복구) + 사유 표면화.
      setStatus(`오류: ${e instanceof Error ? e.message : String(e)}`);
      setPlan(robotId, null);
    } finally {
      setBusy(false);
    }
  };

  const field = (
    label: string,
    k: keyof PoseInput,
    step: number,
    disabled = false,
  ) => (
    <label
      className={`flex items-center gap-1 ${disabled ? "opacity-40" : ""}`}
    >
      <span className="w-10 text-muted-foreground">{label}</span>
      <input
        type="number"
        step={step}
        value={pose[k]}
        disabled={disabled}
        onChange={(e) => setField(k, e.target.value)}
        data-testid={`preview-${k}`}
        className="min-w-0 flex-1 rounded border border-zinc-700 bg-zinc-900 px-1 py-0.5 font-mono"
      />
    </label>
  );

  const jointsReady = !!tcp.value?.joints;

  return (
    <div
      className="flex h-full flex-col gap-3 overflow-y-auto p-3 text-[12px]"
      data-testid="move-preview-panel"
    >
      <section>
        <div className="mb-1 flex items-center justify-between font-mono uppercase text-muted-foreground">
          <span>target position (base frame, m)</span>
          <Button
            size="sm"
            variant="ghost"
            onClick={seedFromCurrent}
            disabled={!jointsReady}
            data-testid="preview-seed"
          >
            현재 위치로
          </Button>
        </div>
        <div className="grid grid-cols-3 gap-x-2 gap-y-1">
          {field("x", "x", 0.01)}
          {field("y", "y", 0.01)}
          {field("z", "z", 0.01)}
        </div>
      </section>

      <section>
        <div className="mb-1 flex items-center gap-2 font-mono uppercase text-muted-foreground">
          <span>자세</span>
          <div className="flex gap-1">
            <Button
              size="sm"
              variant={useOrientation ? "default" : "ghost"}
              onClick={() => setUseOrientation(true)}
              data-testid="preview-ori-on"
            >
              목표 지정
            </Button>
            <Button
              size="sm"
              variant={!useOrientation ? "default" : "ghost"}
              onClick={() => setUseOrientation(false)}
              data-testid="preview-ori-off"
            >
              위치만
            </Button>
          </div>
        </div>
        {useOrientation ? (
          <>
            <p className="mb-1 text-[10px] text-muted-foreground">
              현재 자세 → 이 RPY 로 보간(MoveL) / 도달(MoveJ)
            </p>
            <div className="grid grid-cols-3 gap-x-2 gap-y-1">
              {field("roll°", "roll", 5)}
              {field("pitch°", "pitch", 5)}
              {field("yaw°", "yaw", 5)}
            </div>
          </>
        ) : (
          <p className="text-[10px] text-muted-foreground">
            위치만 도달 — 자세는 IK 가 자유롭게 선택 (도달성↑)
          </p>
        )}
      </section>

      <section>
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          preview (실행 아님 — 고스트만 움직임)
        </div>
        <div className="flex gap-2">
          <Button
            size="sm"
            variant={activeMode === PreviewMode.MOVE_L ? "default" : "outline"}
            onClick={() => runPreview(PreviewMode.MOVE_L)}
            disabled={busy || !jointsReady}
            data-testid="preview-move-l"
          >
            MoveL 프리뷰
          </Button>
          <Button
            size="sm"
            variant={activeMode === PreviewMode.MOVE_J_POSE ? "default" : "outline"}
            onClick={() => runPreview(PreviewMode.MOVE_J_POSE)}
            disabled={busy || !jointsReady}
            data-testid="preview-move-j-pose"
          >
            MoveJ(pose) 프리뷰
          </Button>
        </div>
      </section>

      <section>
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          배속 (재생 속도만 — 슬로모로 wrist flip 관찰)
        </div>
        <div className="flex gap-1">
          {SPEEDS.map((s) => (
            <Button
              key={s}
              size="sm"
              variant={speed === s ? "default" : "ghost"}
              onClick={() => setSpeed(robotId, s)}
              data-testid={`preview-speed-${s}`}
            >
              {s}x
            </Button>
          ))}
        </div>
      </section>

      <div className="mt-auto font-mono text-muted-foreground" data-testid="preview-status">
        {status || (jointsReady ? "pose 입력 후 프리뷰" : "joint state 대기 중…")}
      </div>
    </div>
  );
}
