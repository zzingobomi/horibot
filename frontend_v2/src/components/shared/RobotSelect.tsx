import { useRobots } from "@/hooks/useRobots";

/**
 * robot 선택 드롭다운 — 패널이 소유한 robot 을 고르는 controlled select
 * ([[robot_ownership_model]] §6). 옵션 목록은 /robots (useRobots) 에서 읽는다.
 *
 * picker 가 옵션을 채우려고 robot 목록을 읽는 것은 §4 "환경을 읽지 않는다"(=바인딩
 * 규칙) 위반이 아니다 — picker 는 제 일(선택지 제시)을 할 뿐 바인딩(panel.robot)을
 * 바꾸지 않는다. 실제 바인딩 변경은 onChange 콜백을 받은 쪽(패널)이 자기 상태를
 * 갱신할 때 일어난다.
 */
export function RobotSelect({
  value,
  onChange,
  className = "",
}: {
  value: string | null;
  onChange: (robotId: string) => void;
  className?: string;
}) {
  const { robots } = useRobots();

  return (
    <select
      value={value ?? ""}
      onChange={(e) => onChange(e.target.value)}
      onClick={(e) => e.stopPropagation()}
      onMouseDown={(e) => e.stopPropagation()}
      className={
        "bg-zinc-800 border border-zinc-600/60 rounded text-[10px] font-mono " +
        "text-zinc-200 px-1 py-0.5 focus:outline-none focus:border-zinc-400 " +
        className
      }
    >
      {value == null && (
        <option value="" disabled>
          robot 선택…
        </option>
      )}
      {robots.map((r) => (
        <option key={r.id} value={r.id}>
          {r.id}
        </option>
      ))}
    </select>
  );
}
