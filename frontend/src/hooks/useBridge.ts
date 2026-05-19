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
import type { TaskState } from "@/types/task";
import { useDetectorStore, type Detection } from "@/store/detectorStore";
import { usePointCloudStore } from "@/store/pointCloudStore";

export function useBridge() {
  const setBridgeConnected = useSystemStore((s) => s.setBridgeConnected);
  const updateNode = useSystemStore((s) => s.updateNode);
  const addLog = useSystemStore((s) => s.addLog);
  const setJoints = useRobotStore((s) => s.setJoints);
  const setConfigs = useRobotStore((s) => s.setConfigs);
  const setTorque = useRobotStore((s) => s.setTorque);
  const setJointOffsets = useRobotStore((s) => s.setJointOffsets);
  const setStatus = useCameraStore((s) => s.setStatus);
  const setTrajectoryState = useMotionStore((s) => s.setTrajectoryState);
  const setTaskState = useTaskStore((s) => s.setTaskState);
  const setLoading = useTaskStore((s) => s.setLoading);
  const setDetections = useDetectorStore((s) => s.setDetections);

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

    // Detector 상태 구독
    const unsubDetector = bridge.subscribe(Topic.DETECTOR_STATE, (data) => {
      const { detections, timestamp } = data as {
        detections: Detection[];
        timestamp: number;
      };
      setDetections(detections ?? [], timestamp ?? 0);
    });

    // PointCloud 상태 + 바이너리 스트림 구독
    const unsubPointCloud = usePointCloudStore.getState()._attach();

    // Joint offsets — 캘리브레이션이 추정한 motor zero 보정 (단위: rad).
    // CalibrationNode가 시작 시 1회 + COMMIT 직후 latest-wins로 발행.
    // 프론트엔드는 URDF 적용 시 raw_to_rad에 더해 워크스페이스가 백엔드와 동기화.
    const unsubJointOffsets = bridge.subscribe(
      Topic.CALIB_STATE_JOINT_OFFSETS,
      (data) => {
        const { offsets } = data as {
          offsets: { motor_id: number; offset_rad: number }[];
        };
        const map: Record<number, number> = {};
        for (const e of offsets ?? []) map[e.motor_id] = e.offset_rad;
        setJointOffsets(map);
      }
    );

    return () => {
      unsubJoint();
      unsubHeartbeat();
      unsubLog();
      unsubCameraStatus();
      unsubTraj();
      unsubTask();
      unsubDetector();
      unsubPointCloud();
      unsubJointOffsets();
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
    setJointOffsets,
    setTrajectoryState,
    setTaskState,
    setLoading,
    setDetections,
  ]);
}
