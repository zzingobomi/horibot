import yaml from "js-yaml";
import { BASE_URL } from "@/constants";

export interface JointAngle {
  id: number;
  degree: number;
}

const cache = new Map<string, Record<string, JointAngle[]>>();

async function loadAll(
  robotId: string,
): Promise<Record<string, JointAngle[]>> {
  const cached = cache.get(robotId);
  if (cached !== undefined) return cached;

  const url = `${BASE_URL}/robot/instances/${robotId}/robot_poses.yaml`;
  const res = await fetch(url);
  if (!res.ok) {
    throw new Error(
      `robot_poses.yaml fetch 실패 (${res.status}, robot=${robotId})`,
    );
  }
  const text = await res.text();
  const raw = yaml.load(text);

  if (!raw || typeof raw !== "object") {
    throw new Error(`robot_poses.yaml: dict 아님 (robot=${robotId})`);
  }

  const parsed: Record<string, JointAngle[]> = {};
  for (const [name, jointsRaw] of Object.entries(raw as Record<string, unknown>)) {
    if (!Array.isArray(jointsRaw) || jointsRaw.length === 0) {
      throw new Error(
        `robot_poses.yaml: '${name}' joints 리스트 아님 (robot=${robotId})`,
      );
    }
    parsed[name] = jointsRaw.map((j) => {
      if (
        typeof j !== "object" ||
        j === null ||
        !("id" in j) ||
        !("degree" in j)
      ) {
        throw new Error(
          `robot_poses.yaml: '${name}' 잘못된 항목 ${JSON.stringify(j)} (robot=${robotId})`,
        );
      }
      return {
        id: Number((j as { id: unknown }).id),
        degree: Number((j as { degree: unknown }).degree),
      };
    });
  }

  cache.set(robotId, parsed);
  return parsed;
}

export async function loadPose(
  robotId: string,
  name: string,
): Promise<JointAngle[]> {
  const all = await loadAll(robotId);
  const pose = all[name];
  if (!pose) {
    throw new Error(
      `robot_poses.yaml 에 '${name}' 자세 없음 (robot=${robotId}). 사용 가능: ${Object.keys(all).join(", ")}`,
    );
  }
  return pose;
}
