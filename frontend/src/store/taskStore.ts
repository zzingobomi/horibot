import { create } from "zustand";
import type { TaskState, TaskTree } from "@/types/task";
import { defaultTaskState, defaultTaskTree } from "@/types/task";

interface TaskStore {
  taskState: TaskState;
  taskTree: TaskTree;
  loading: boolean;
  setTaskState: (s: TaskState) => void;
  setTaskTree: (t: TaskTree) => void;
  setLoading: (v: boolean) => void;
}

export const useTaskStore = create<TaskStore>((set) => ({
  taskState: defaultTaskState,
  taskTree: defaultTaskTree,
  loading: false,
  setTaskState: (s) => set({ taskState: s }),
  setTaskTree: (t) => set({ taskTree: t }),
  setLoading: (v) => set({ loading: v }),
}));
