"""link_offsets.npz 에 tool_offset (ID=6) 행을 추가/갱신하는 1회 패치 스크립트.

배경
----
URDF 의 end_effector_joint xyz="0.09193 -0.0016 0" 가 실제 그리퍼 끝점 (핑거 닫혔을
때 만나는 점) 보다 link5 +x 방향으로 약 +11mm 길게 정의돼있음. 측정: 2026-05-28
search_1 자세 2회 grasp 결과 모두 갭 ~+11mm 일관 (큐브 상단을 침).

설계
----
URDF EE 자체는 *그대로 두고* (캘리브레이션 / IK 의 reference frame 안정성 유지),
link_offsets ID=6 행을 *tool_offset 으로 재해석*:
    실제 그리퍼 끝점 = URDF EE + R_be @ tool_offset_ee

이 값을 motion_node 의 service handler 가 사용:
    - _srv_get_tcp: URDF EE pose 응답에 tool_offset 더해서 user frame 반환
    - _srv_move_l / _srv_move_tcp / _cartesian_handler: user 명령 좌표에서 tool_offset
      빼서 URDF target 도출 → IK
이렇게 detect 와 IK 가 *다른 시점에 한쪽씩만* 변환 → cancel out 회피.

(이력: 한때 urdf_patcher 가 ID=6 을 URDF patch 로 적용 시도. detect 와 IK 가 같은
patched URDF 위에서 도는 self-consistency 로 cancel out → 효과 0. 22:18 실측 확인.
fix: urdf_patcher 는 ID=6 무시, motion_node 만 tool_offset 으로 사용.)

ID=6 의 namespace
-----------------
모터 ID 6 = 그리퍼와 *수치만* 같음. link_offsets 는 모터 무관 — 여기서 ID=6 은
"tool_offset 행" 의미. 모터/그리퍼 코드와 분리.

적용
----
1. uv run --project backend python backend/patch_ee_link_offset.py
2. PC + 모터 Pi 백엔드 재시작
   (urdf_patcher 는 ID=6 무시하므로 patched URDF 는 변경 X, 단 motion_node 가
    재시작 시 LinkCoordinates 로드)
3. pick_and_place 다시 시도 → 큐브 옆면 가운데 잡는지 검증

자세 의존성
-----------
tool_offset_ee 는 EE frame 기준 — 각 자세의 R_be 로 자동 변환되므로 *모든 자세에서*
일관 적용. search_1 한 자세 측정값이지만 link5 → 실제 끝점 거리는 자세 무관 (rigid
부착) 이라 +11mm 추정이 정확하면 다른 자세에서도 fix.

5DOF position-only IK 한계: trajectory 보간 중 자세 변화 시 그 점의 R 이 시작점 R 과
다를 수 있음. 짧은 trajectory (< 0.1m) 에서는 무시 가능. 더 정확히는 _tool_offset_base
를 trajectory_runner 의 매 IK 호출 시점에서 다시 계산해야 함 (현재는 service handler
진입 시점 R 한 번 사용).
"""

from pathlib import Path

import numpy as np


LINK_OFFSETS_PATH = (
    Path(__file__).parent.parent / "robot" / "calibration" / "link_offsets.npz"
)
TOOL_OFFSET_LINK_ID = 6
# EE frame 기준 (실제 그리퍼 끝점 - URDF EE) 벡터.
# URDF EE x = 91.93mm. 실측 추정: 실제 끝점 = link5 +x 방향으로 80.93mm
# → tool_offset_ee.x = 80.93 - 91.93 = -11mm
TOOL_TRANS_M = np.array([-0.011, 0.0, 0.0], dtype=np.float64)
TOOL_ROT_RAD = np.array([0.0, 0.0, 0.0], dtype=np.float64)


def main() -> None:
    if not LINK_OFFSETS_PATH.exists():
        print(f"[ERROR] link_offsets.npz 없음: {LINK_OFFSETS_PATH}")
        return

    data = np.load(str(LINK_OFFSETS_PATH), allow_pickle=False)
    ids = data["joint_ids"].astype(int).tolist()
    trans = data["link_trans_m"].astype(np.float64)
    rot = data["link_rot_rad"].astype(np.float64)
    method = str(data["method"]) if "method" in data.files else "manual+tool_offset"

    print(f"기존: ids={ids}")
    print(f"기존 trans shape={trans.shape}")

    if TOOL_OFFSET_LINK_ID in ids:
        idx = ids.index(TOOL_OFFSET_LINK_ID)
        old_trans = trans[idx].copy()
        trans[idx] = TOOL_TRANS_M
        rot[idx] = TOOL_ROT_RAD
        print(f"[UPDATE] ID={TOOL_OFFSET_LINK_ID} 행 갱신: trans {old_trans} → {TOOL_TRANS_M}")
    else:
        ids.append(TOOL_OFFSET_LINK_ID)
        trans = np.vstack([trans, TOOL_TRANS_M[None, :]])
        rot = np.vstack([rot, TOOL_ROT_RAD[None, :]])
        print(f"[ADD] ID={TOOL_OFFSET_LINK_ID} 행 추가: trans={TOOL_TRANS_M.tolist()}")

    new_method = method if "tool_offset" in method else method + "+tool_offset"

    np.savez(
        str(LINK_OFFSETS_PATH),
        joint_ids=np.array(ids, dtype=np.int32),
        link_trans_m=trans,
        link_rot_rad=rot,
        method=new_method,
    )
    print(f"\n저장됨: {LINK_OFFSETS_PATH}")
    print(f"신규: ids={ids}, trans shape={trans.shape}")
    print("\n다음 단계:")
    print("  1. PC + 모터 Pi 모두 백엔드 재시작 (motion_node 가 LinkCoordinates 재로드)")
    print("  2. .patched/omx_f.urdf 는 변경 X — urdf_patcher 가 ID=6 무시")
    print("  3. pick_and_place 시도 → 큐브 옆면 가운데 잡는지 검증")


if __name__ == "__main__":
    main()
