import { useEffect } from "react";
import { bridge } from "@/api/bridge";
import { useSystemStore } from "@/store/systemStore";
import { ServiceKey, Topic } from "@/constants/topics";
import type { JointState, MotorConfig } from "@/types/motor";
import { useRobotStore } from "@/store/robotStore";
import type { CameraStatus } from "@/types/camera";
import { useCameraStore } from "@/store/cameraStore";
import { useMotionStore } from "@/store/motionStore";
import type { TrajectoryState } from "@/types/motion";
import { useTaskStore } from "@/store/taskStore";
import type { TaskState, TaskTree } from "@/types/task";
import { useDetectorStore, type Detection } from "@/store/detectorStore";
import { usePointCloudStore } from "@/store/pointCloudStore";
import { useTaskResultStore, type StepResultPayload } from "@/store/taskResultStore";

export function useBridge() {
  const setBridgeConnected = useSystemStore((s) => s.setBridgeConnected);
  const updateNode = useSystemStore((s) => s.updateNode);
  const addLog = useSystemStore((s) => s.addLog);
  const setJoints = useRobotStore((s) => s.setJoints);
  const setConfigs = useRobotStore((s) => s.setConfigs);
  const setTorque = useRobotStore((s) => s.setTorque);
  const setStatus = useCameraStore((s) => s.setStatus);
  const setTrajectoryState = useMotionStore((s) => s.setTrajectoryState);
  const setTaskState = useTaskStore((s) => s.setTaskState);
  const setTaskTree = useTaskStore((s) => s.setTaskTree);
  const setLoading = useTaskStore((s) => s.setLoading);
  const setDetections = useDetectorStore((s) => s.setDetections);
  const setGroundedResult = useDetectorStore((s) => s.setGroundedResult);
  const setStepResult = useTaskResultStore((s) => s.setStepResult);
  const clearStepResults = useTaskResultStore((s) => s.clearAll);

  useEffect(() => {
    // Bridge 연결
    bridge.connect((connected) => {
      setBridgeConnected(connected);
      if (connected) {
        // 연결되면 모터 설정 정보 요청
        bridge.callService(ServiceKey.MOTOR_GET_CONFIG, {}).then((res) => {
          if (res.success && res.data?.motors) {
            setConfigs(res.data.motors as MotorConfig[]);
            console.log(res.data);
            if (res.data.torque_enabled !== undefined) {
              setTorque(res.data.torque_enabled as boolean);
            }
          }
        });
      }
    });

    // Joint 상태 구독
    const unsubJoint = bridge.subscribe(Topic.MOTOR_STATE_JOINT, (data) => {
      const state = data as unknown as JointState;
      setJoints(state.joints ?? []);
    });

    // Heartbeat 구독
    const unsubHeartbeat = bridge.subscribe(Topic.SYSTEM_HEARTBEAT, (data) => {
      const { node, status, timestamp } = data as {
        node: string;
        status: string;
        timestamp: number;
      };
      updateNode(node, status === "ok" ? "running" : "error", timestamp);
    });

    // 로그 구독
    const unsubLog = bridge.subscribe(Topic.SYSTEM_LOG, (data) => {
      addLog(
        data as {
          timestamp: number;
          node: string;
          level: string;
          message: string;
        }
      );
    });

    // 카메라 상태 구독
    const unsubCameraStatus = bridge.subscribe(
      Topic.CAMERA_STATE_STATUS,
      (data) => {
        setStatus(data as unknown as CameraStatus);
      }
    );

    // Trajectory 상태 구독
    const unsubTraj = bridge.subscribe(Topic.MOTION_STATE_TRAJ, (data) => {
      setTrajectoryState(data as unknown as TrajectoryState);
    });

    // Task 상태 구독
    const unsubTask = bridge.subscribe(Topic.TASK_STATE, (data) => {
      const state = data as unknown as TaskState;
      setTaskState(state);
      if (state.status !== "idle") setLoading(false);
    });

    // Task tree 구독 — task 시작 시 1회 publish, latest-wins 큐로 늦게 붙은
    // 클라이언트도 마지막 tree 받음. 새 tree 면 누적된 step result 도 클리어.
    const unsubTaskTree = bridge.subscribe(Topic.TASK_TREE, (data) => {
      clearStepResults();
      setTaskTree(data as unknown as TaskTree);
    });

    // Step result 구독 — 각 step 완료 시 1회 publish. type 별로 다른 렌더러
    // 가 dispatch (Detection→sphere, Position3→marker, ...).
    const unsubStepResult = bridge.subscribe(
      Topic.TASK_STEP_RESULT,
      (data) => {
        setStepResult(data as unknown as StepResultPayload);
      },
    );

    // Detector 상태 구독
    const unsubDetector = bridge.subscribe(Topic.DETECTOR_STATE, (data) => {
      const { detections, timestamp } = data as {
        detections: Detection[];
        timestamp: number;
      };
      setDetections(detections ?? [], timestamp ?? 0);
    });

    // Grounded detection 결과 broadcast 구독 (호출자 무관, 일관 시각화)
    const unsubGrounded = bridge.subscribe(
      Topic.PERCEPTION_GROUNDED_STATE,
      (data) => {
        const r = data as {
          prompt?: string;
          position?: [number, number, number];
          bbox2d?: { x1: number; y1: number; x2: number; y2: number };
          confidence?: number;
          timestamp?: number;
        };
        if (r.prompt && r.position && r.bbox2d && r.confidence != null) {
          setGroundedResult({
            prompt: r.prompt,
            position: r.position,
            bbox2d: r.bbox2d,
            confidence: r.confidence,
            timestamp: r.timestamp ?? Date.now(),
          });
        }
      },
    );

    // PointCloud 상태 + 바이너리 스트림 구독
    const unsubPointCloud = usePointCloudStore.getState()._attach();

    // joint_offsets는 useCalibrationResults가 HTTP fetch로 처리 (토픽 폐기).

    return () => {
      unsubJoint();
      unsubHeartbeat();
      unsubLog();
      unsubCameraStatus();
      unsubTraj();
      unsubTask();
      unsubTaskTree();
      unsubStepResult();
      unsubDetector();
      unsubGrounded();
      unsubPointCloud();
      bridge.disconnect();
    };
  }, [
    setBridgeConnected,
    updateNode,
    addLog,
    setJoints,
    setConfigs,
    setStatus,
    setTorque,
    setTrajectoryState,
    setTaskState,
    setTaskTree,
    setLoading,
    setDetections,
    setGroundedResult,
    setStepResult,
    clearStepResults,
  ]);
}
