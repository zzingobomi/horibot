# Frontend URDF visual ↔ backend corrected TCP mismatch (2026-06-22, 다음 세션 논의 anchor)

> SO-101 + D405 setup 에서 3D viewer 의 *URDF tcp link visual* (빨간 box) 와
> *TCP 좌표축* (label="TCP", backend `MOTION_STATE_TCP`) 가 위치 어긋남. ≤ 4°
> 정도, 시각 cosmetic 만 — robot 명령 / 캘 / motion 다 좌표축 (corrected FK)
> 기준이라 동작 영향 0. 사용자가 "URDF 잘못 맞춘 거 아니냐" 로 인지해 발견.
> 진단 끝, 수정 보류 (추후 논의 후 진행).

## 증상

[Move 페이지 / Calibrate 페이지 공통] 3D viewer 에서:
- URDF 의 `<link name="tcp">` visual (작은 빨간 box, [so101_6dof.urdf:448-454](../robot/so101_6dof/urdf/so101_6dof.urdf#L448-L454)) 가 한 자리
- TCP 좌표축 (X/Y/Z arrow + "TCP" label) 이 그 옆 살짝 떨어진 자리
- 둘이 정확히 겹쳐야 *시각적으로 일관* 하지만 어긋남

## Root cause — 두 FK chain 이 서로 다른 URDF + sag 유무

같은 joint angle (backend `MOTOR_STATE_JOINT`) 을 입력으로 받지만:

```
joint angle ─┬─→ frontend urdf-loader (RobotModel.tsx)
             │     ├─ 입력: /robot/.../so101_6dof.urdf  (정적 마운트 raw 원본)
             │     ├─ sag 보정: 없음
             │     └─ 결과: tcp link 위치 → 빨간 box 렌더
             │
             └─→ backend motion_node
                   ├─ 입력: 같은 URDF 를 in-memory patch (link_offset 적용)
                   │       pybullet_kinematics.py:96 `patch_urdf_text`
                   ├─ sag 보정: SagCorrectedKinematics Decorator (J2/J3 처짐)
                   └─ 결과: MOTION_STATE_TCP publish → 좌표축 렌더
```

차이:
| | URDF 자체 | sag |
|---|---|---|
| 빨간 박스 (frontend) | raw 원본 | ❌ |
| 좌표축 (backend) | link_offset patch | ✅ |

캘 5종 모두 active (`storage/horibot.db::calibration_results WHERE is_active=1` —
joint_offset / link_offset / sag / hand_eye / intrinsic) 라 mismatch 가 visible.

## 왜 frontend 가 patched URDF 를 못 쓰나

- backend [pybullet_kinematics.py:103-122](../backend/modules/kinematics/adapters/pybullet_kinematics.py#L103-L122) 가 patch 결과를 `tempfile` 로 1회성 write → `loadURDF` → `unlink`. 디스크에 안 남김.
- frontend urdf-loader 는 [bridge/zenoh_bridge.py](../backend/bridge/zenoh_bridge.py) 가 `/robot` 으로 정적 마운트한 **raw 파일** HTTP fetch — patch 가 안 박힌 원본.
- sag 는 자세 의존 보정 (Decorator) 이라 *URDF 수정으로 표현 불가능* — frontend FK 에 넣을 방법 자체가 없음 (joint angle 마다 다른 보정량).

## Impact

| | 영향 |
|---|---|
| MoveL / MoveC / MoveP / ServoTcp / JogTcp 도달점 | ✅ 좌표축 기준 (corrected FK) — 빨간 박스 무시 |
| Hand-eye 카메라 frustum / cameraMatrix | ✅ 좌표축 기준 |
| PyBullet IK target | ✅ 좌표축 기준 |
| Live PointCloud transform | ✅ 좌표축 기준 (`cameraMatrix = tcpMatrix · handEye`) |
| 빨간 박스 위치 | ❌ raw URDF FK — 시각 표시에만 사용, 명령 chain 어디에도 안 들어감 |

→ **시각 cosmetic 만**. critical X. ([Container.tsx:72-76](../frontend/src/components/scene/Container.tsx#L72-L76) 에 "critical 아니라 미룸" 으로 박혀있는 자리.)

## Fix 옵션 (보류, 추후 논의)

### 옵션 A: bridge 가 patched URDF 를 endpoint 로 서빙

- bridge 에 `GET /robots/{robot_id}/urdf` 추가 — `RobotRegistry.get_kinematics(id)` 가 들고 있는 patched URDF 텍스트 (또는 path) 반환
- frontend urdf-loader 가 정적 `/robot/<type>/urdf/...` 대신 robot 별 patched endpoint fetch
- 단 sag 는 여전히 안 들어감 → ≤ 4° 잔존 (자세 따라 변동)
- **장점**: 구현 간단. URDF 가 robot instance 의 link_offset 반영
- **단점**: sag mismatch 잔존. urdf-loader 가 robot 별 URDF 받는 lifecycle 변경 필요

### 옵션 B: backend 가 전체 link pose topic publish + frontend urdf-loader 자체 FK off

- backend 가 `MOTION_STATE_LINKS` (또는 비슷한 이름) topic 으로 *각 link 의 corrected world pose* 매 motor update 마다 publish (joint state 기반)
- frontend `RobotModel.tsx` 가 urdf-loader 의 자체 FK 끄고 각 link 를 backend pose 로 직접 setMatrix
- **장점**: 완전 일관. sag 도 반영. *진짜* SSOT 정석 (캘/sag 가 backend 한 곳에만 산다는 본 architecture 원칙과 align)
- **단점**: urdf-loader 의 자체 FK 끄는 API hook 필요 (gkjohnson loader 가 지원하는지 확인). bandwidth 살짝 증가 (link 수 × matrix 4×4). RobotModel 자리 자체 FK chain 다 갈아엎기

### 옵션 C: 그대로 두기 (현재)

- Container.tsx 코멘트에 박혀있듯 "≤ 4°, critical 아님"
- 캘 정확도 / 명령 / 모션 다 backend SSOT, 빨간 박스는 frontend cosmetic
- **단점**: 새 사용자/개발자가 매번 헷갈림 (사용자가 "URDF 잘못 맞춘 거 아니냐" 로 발견). 학습 곡선 비용

## 결정 사항 (추후 논의 항목)

1. 옵션 A / B / C 중 선택
2. 옵션 B 의 경우 — urdf-loader 가 외부 link pose 받는 API 존재 여부 (gkjohnson urdf-loaders 문서 확인 자리)
3. 옵션 A 의 경우 — patched URDF 가 robot instance 별 다르므로 URL scheme 결정 (`/robots/{id}/urdf` vs `/robot/instances/{id}/urdf` 등)
4. 빨간 box 자체 제거 옵션 — `<link name="tcp">` 의 `<visual>` 빼고 backend 좌표축만 시각에 남기는 것. 가장 간단하지만 "URDF 에 grasp point 표시" 의도 사라짐

## 관련 코드 위치

- Backend FK (corrected): [pybullet_kinematics.py](../backend/modules/kinematics/adapters/pybullet_kinematics.py) + [sag_corrected.py](../backend/modules/kinematics/adapters/sag_corrected.py)
- URDF patch: [urdf_patcher.py](../backend/core/coords/urdf_patcher.py) + `patch_urdf_text`
- TCP topic publish: [motion_node.py:580-595](../backend/nodes/device/motion_node.py#L580) (`_publish_tcp_loop`)
- Frontend tcpMatrix 수신: [Container.tsx:77-107](../frontend/src/components/scene/Container.tsx#L77-L107)
- Frontend URDF FK (raw): [RobotModel.tsx](../frontend/src/components/canvas/3d/RobotModel.tsx) (gkjohnson urdf-loader)
- 정적 URDF 마운트: bridge 의 `app.mount("/robot", StaticFiles(directory=...))`

## 관련 문서

- [calibration_apply_flow.md](calibration_apply_flow.md) — 캘 4종이 어디서 적용되는지 (link_offset 의 URDF in-memory patch 가 backend 한정인 사실 자리)
- [multi_robot_architecture.md](multi_robot_architecture.md) §3.1 — Kinematics layer decorator chain
- [move_page_pointcloud_issues.md](move_page_pointcloud_issues.md) #5 — *별개* 이슈 (URDF joint limits clip 으로 인한 URDF FK 자체 오류). 본 이슈는 limit 가 아니라 link_offset+sag 차이라 root cause 다름.
