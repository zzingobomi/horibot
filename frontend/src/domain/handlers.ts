/**
 * 도메인 비즈니스 — 토픽 도착 시 *누적 / side-effect* 자리만.
 *
 * App.tsx 가 `import "@/domain/handlers"` 1줄로 마운트. 단순 latest cache 토픽
 * (motor/camera/motion/task/detector state) 는 framework store 가 자동 흡수
 * 하므로 본 파일에 등재 X — 사용처는 `useTopic(Topic.X)` 직접.
 *
 * 새 비즈니스 추가 = 본 파일 1줄. cast / 가드 보일러플레이트 0 —
 * `onTopic<K>` 가 generated `TopicPayloadMap[K]` 로 자동 typed.
 */
import { onConnect, onTopic } from "@/framework";
import { Topic } from "@/constants/topics";
import { useSystemStore } from "@/domain/stores/system";
import { useTaskResultStore, type StepResultPayload } from "@/domain/stores/taskResult";
import { usePointCloudStore } from "@/domain/stores/pointCloud";

// 재연결 시 멱등 — 이전 unsub 호출 후 재attach.
let unsubPointCloud: (() => void) | null = null;

onConnect(() => {
  // MOTOR_GET_CONFIG prefetch 는 useFrameworkBootstrap 가 robot-explicit 으로
  // 처리 (robots loaded + WS connected 둘 다 충족 시점). 여기선 bridge default
  // robot 가 race 로 wrong id 가 될 수 있어 호출 X.

  // PointCloud 는 binary 토픽 — bootstrap 가 BINARY_TOPICS 는 skip 하므로 store 자체 attach.
  if (unsubPointCloud) unsubPointCloud();
  unsubPointCloud = usePointCloudStore.getState()._attach();
});

onTopic(Topic.SYSTEM_HEARTBEAT, (hb) => {
  useSystemStore.getState().updateNode(
    hb.node,
    hb.status === "ok" ? "running" : "error",
    hb.timestamp,
    hb.robot_id ?? null,
  );
});

onTopic(Topic.SYSTEM_LOG, (log) => {
  useSystemStore.getState().addLog(log);
});

onTopic(Topic.TASK_TREE, () => {
  // 새 task tree 도착 = 누적 step result 클리어 (시각화 깨끗이).
  useTaskResultStore.getState().clearAll();
});

onTopic(Topic.TASK_STEP_RESULT, (data) => {
  useTaskResultStore.getState().setStepResult(data as StepResultPayload);
});
