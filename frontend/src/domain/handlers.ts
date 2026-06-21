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
import { bridge } from "@/api/bridge";
import { useSystemStore } from "@/domain/stores/system";
import { useTaskResultStore, type StepResultPayload } from "@/domain/stores/taskResult";
import { useScene3DStore } from "@/domain/stores/scene3D";

// Scene3D 는 binary 토픽 — bootstrap useEffect(2) 가 BINARY_TOPICS 자리 skip 하므로
// store 자체 attach. 단 robot-scoped 라 bridge.defaultRobotId 변경 시 *re-attach
// 필수* — 안 두면 첫 connect 시점의 옛 DEFAULT_ROBOT_ID (constants hardcoded)
// 로 영구 sub → 실 robot publish 미수신 (PointCloud 안 보이는 회귀).
let unsubScene3D: (() => void) | null = null;
function _reattachScene3D(): void {
  if (unsubScene3D) unsubScene3D();
  unsubScene3D = useScene3DStore.getState()._attach();
}

onConnect(_reattachScene3D);
bridge.onDefaultRobotIdChange(_reattachScene3D);

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
