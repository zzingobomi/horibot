import yaml from "js-yaml";
import { BASE_URL } from "@/constants";

export interface JointAngle {
  id: number;
  degree: number;
}

const ROBOT_POSES_URL = `${BASE_URL}/robot/config/robot_poses.yaml`;

let cache: Record<string, JointAngle[]> | null = null;

async function loadAll(): Promise<Record<string, JointAngle[]>> {
  if (cache !== null) return cache;

  const res = await fetch(ROBOT_POSES_URL);
  if (!res.ok) {
    throw new Error(`robot_poses.yaml fetch 실패 (${res.status})`);
  }
  const text = await res.text();
  const raw = yaml.load(text);

  if (!raw || typeof raw !== "object") {
    throw new Error("robot_poses.yaml: dict 아님");
  }

  const parsed: Record<string, JointAngle[]> = {};
  for (const [name, jointsRaw] of Object.entries(raw as Record<string, unknown>)) {
    if (!Array.isArray(jointsRaw) || jointsRaw.length === 0) {
      throw new Error(`robot_poses.yaml: '${name}' joints 리스트 아님`);
    }
    parsed[name] = jointsRaw.map((j) => {
      if (
        typeof j !== "object" ||
        j === null ||
        !("id" in j) ||
        !("degree" in j)
      ) {
        throw new Error(
          `robot_poses.yaml: '${name}' 잘못된 항목 ${JSON.stringify(j)}`,
        );
      }
      return {
        id: Number((j as { id: unknown }).id),
        degree: Number((j as { degree: unknown }).degree),
      };
    });
  }

  cache = parsed;
  return cache;
}

export async function loadPose(name: string): Promise<JointAngle[]> {
  const all = await loadAll();
  const pose = all[name];
  if (!pose) {
    throw new Error(
      `robot_poses.yaml 에 '${name}' 자세 없음. 사용 가능: ${Object.keys(all).join(", ")}`,
    );
  }
  return pose;
}
