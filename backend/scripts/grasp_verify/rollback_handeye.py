"""hand_eye 보정 롤백 — 6/21 원본(offline_BA_stage_D)을 다시 활성화.
파생 보정 row 는 is_active=0 으로 비활성 (삭제 아님, 이력 보존)."""
import sqlite3

ROBOT = "so101_6dof_0"
c = sqlite3.connect("horibot.db")
rows = c.execute("SELECT id,created_at,is_active,result_data FROM calibration_results WHERE robot_id=? AND kind='hand_eye' ORDER BY id", (ROBOT,)).fetchall()
print("hand_eye rows:")
for r in rows:
    method = ""
    try:
        import json; method = json.loads(r[3]).get("method","")[:40]
    except Exception: pass
    print(f"  id={r[0]} active={r[2]} {r[1][:19]} {method}")
# 원본 = offline_BA_stage_D
orig = None
for r in rows:
    import json
    if "offline_BA_stage_D" in (json.loads(r[3]).get("method","")):
        orig = r[0]
if orig is None:
    print("원본 offline_BA_stage_D hand_eye 를 못 찾음 — 수동 확인 필요"); raise SystemExit(1)
c.execute("UPDATE calibration_results SET is_active=0 WHERE robot_id=? AND kind='hand_eye'", (ROBOT,))
c.execute("UPDATE calibration_results SET is_active=1 WHERE id=?", (orig,))
c.commit()
print(f"\n롤백 완료 → id={orig} (6/21 원본) 활성화. PC backend 재시작 필요.")
