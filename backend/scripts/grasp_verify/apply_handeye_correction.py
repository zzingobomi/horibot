"""보정된 hand_eye 를 새 calibration_results row 로 추가 + 활성화.
기존 활성 hand_eye 는 보존(is_active=0)만 — 롤백은 rollback_handeye.py.
object-anywhere 안전: 카메라 외부변환(hand_eye)만 교체. 지지면 가정 없음."""
import sqlite3, json
from datetime import datetime, timezone

ROBOT = "so101_6dof_0"
corr = json.load(open("debug/handeye_correction.json", encoding="utf-8"))
result_data = json.dumps({
    "R_cam2gripper": corr["R_cam2gripper"],
    "t_cam2gripper": corr["t_cam2gripper"],
    "method": "derived_correction_from_20260715_detect (table-flat + cube-consistency)",
})
now = datetime.now(timezone.utc).isoformat(sep=" ")

c = sqlite3.connect("horibot.db")
old = c.execute("SELECT id,run_id FROM calibration_results WHERE robot_id=? AND kind='hand_eye' AND is_active=1", (ROBOT,)).fetchone()
print("기존 활성 hand_eye:", old)
c.execute("UPDATE calibration_results SET is_active=0 WHERE robot_id=? AND kind='hand_eye'", (ROBOT,))
cur = c.execute(
    "INSERT INTO calibration_results (run_id,robot_id,kind,created_at,is_active,result_data) VALUES (?,?,?,?,1,?)",
    (old[1] if old else None, ROBOT, "hand_eye", now, result_data),
)
c.commit()
print(f"새 hand_eye id={cur.lastrowid} 활성화 (δR≈{corr['delta_R_deg']:.1f}°, δt≈{[round(x,1) for x in corr['delta_t_mm']]}mm)")
print("\n다음: PC backend 재시작(캘 reload) → 라이브 클라우드에서 테이블이 평평해졌는지 확인 → 큐브 파지 테스트.")
print("롤백: .venv\\Scripts\\python.exe scripts\\grasp_verify\\rollback_handeye.py")
