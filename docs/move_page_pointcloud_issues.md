# Move 페이지 Live PointCloud 이슈 정리 (2026-06-21, 다음 세션 anchor)

> SO-101 + D405 setup 자리 frontend Move 페이지의 PointCloud 토글로 실시간 point
> cloud 확인 시도 중 발견된 issue 5 가지. 사용자 피곤해서 진단만 끝, 수정은 다음 세션.

---

## 2026-06-22 세션 update — 진척 + disprove + 다음 진입점

### 이번 세션 한 것

1. **PyBullet self-collision 적용** (별개 task) — `PybulletKinematics` 의 `URDF_USE_SELF_COLLISION` flag + `self_collision()` 실 구현 + `ik()` 통합. SO-101 URDF 의 18개 visual 을 collision 으로 mirror. live PC issue 와 무관, 안전망 추가
2. **factory_intrinsic retry 패턴 fix** — motor 와 비대칭 해소. 처음에 async wrapper 도입했다가 사용자 지적으로 다시 blocking 으로 (storage = essential 인프라). `seed_d405_intrinsic_to_storage` 가 `load_active_blocking` 사용, `camera_node.start()` 에서 `self.camera.open()` 직전 호출. 사용자가 처음에 "캘 못 읽어와서 그런 거 아냐?" 라고 의심한 자리 — 일단 fix 들어감

### Disprove — "cal-fetching 이 root cause" 는 empirical 으로 무너짐

- 사용자가 한 번 pi_camera 재시작 + cal fetch 후 → live PC 동작
- 그러나 *재차 시도 시 다시 안 됨* — cal 이 root cause 라면 fix 후 안정적이어야 함, 아니므로 **cal hypothesis 폐기**
- factory_intrinsic retry fix 자체는 valid (motor 비대칭 해소) 라 revert 안 함. 단 *live PC issue 와는 무관*

### 강력한 의심 — Pi 의 leftover process / 좀비

- 첫 시도 동작 ↔ 두 번째 안 됨 = 어떤 *intermittent state* 가 시스템에 누적됨
- 사용자 instinct: pi_camera 를 ctrl+C 로 끈 뒤 *child python 이 좀비로 남아* RealSense USB / Zenoh queryable 을 계속 점유
- [[feedback-uv-run-subprocess-orphan]] 의 동형 패턴 (Windows 사례) — Linux 에선 검증 안 됨, 하지만 uv wrapper + signal 핸들링 한계로 가능성 충분
- [[feedback-distributed-broadcast-affects-real-robot]] 와 결합 — 같은 LAN 의 좀비 process 가 zenoh peer 로 살아있으면 stale queryable / 중복 publish 유발

### 다음 세션 첫 작업 — Linux 진단 명령어

라즈베리파이 `hori2` SSH 접속 후 (camera Pi):

```bash
# 1. python process 다 보기 (현재 + 좀비)
ps aux | grep -E 'python|horibot' | grep -v grep
pgrep -af python

# 2. zenoh default port (7447) 점유 확인
sudo lsof -iTCP:7447 -sTCP:LISTEN
ss -tnlp | grep 7447

# 3. RealSense USB device 점유 확인
lsusb | grep -i intel
sudo lsof | grep -i realsense
sudo fuser -v /dev/bus/usb/*/* 2>/dev/null | grep -v '^$'

# 4. 좀비 process
ps -eo pid,ppid,stat,comm | awk '$3 ~ /Z/ {print}'

# 5. uv 의 child python pid — uv wrapper 가 죽어도 child 살아있나
ps --ppid 1 | grep python    # init 으로 reparent 된 orphan
ps -eo pid,ppid,etime,comm | grep python
```

### 만약 좀비 / orphan 확인되면

현재 `shutdown` 핸들러 ([main.py:194](../backend/main.py#L194)) 가 `sys.exit(0)` 로 force exit → daemon thread 가 abort 되면서 RealSense USB / zenoh socket 을 잡힌 채로 죽을 가능성. fix 후보:

- signal 핸들러가 RealSense pipeline 명시적 close + zenoh session close 까지 *완료 wait*
- daemon thread 가 아니라 non-daemon + 명시적 join (단 main 종료 보장 빡빡해짐)
- `atexit` 핸들러로 RealSense / zenoh cleanup 이중 보장

### 의심 ranking 재배치 (2026-06-22)

| # | 항목 | 이번 세션 update |
|---|---|---|
| 1 | URDF joint limits 너무 좁음 | 미수정 — 사선 root cause 후보 (cloud 보일 때) |
| 2 | DEFAULT_ROBOT_ID hardcoded | 미수정 |
| 3 | React infinite loop on toggle | 미수정 — *intermittent 패턴* 의 일부일 가능성 |
| 4 | Zenoh stale queryable on PC restart | **★★★ 격상** — leftover process 가 이걸 유발할 가능성 |
| 5 | Live PC 사선 | (#1 의존) |
| 신규 | **pi_camera 좀비 process (의심)** | **★★★ 신규** — Linux 진단 1순위 |

### 검증 안 된 self-warn

- "ctrl+C → cal fetch → 동작" 의 cal fetch 가 실제 무엇이었는지 사용자가 명시 안 함. PC backend log 도 없음
- "또 안 되네" 의 정확한 symptom (안 뜸 / freeze / 사선 / 기타) 명시 안 함 — 다음 세션 첫 질문 자리
- factory_intrinsic retry fix 가 *어쩌면* live PC 와 *간접* 연관 가능 (storage 가 늦게 떠도 fix 후 자동 retry → 다른 노드의 cal load 도 결국 채워짐). 그러나 *intermittent 자체* 는 설명 X

---

## Critical issue 우선순위

| # | 항목 | 시급도 | 코드 vs 데이터 |
|---|---|---|---|
| 1 | **URDF joint limits 너무 좁음** | ★★★ | URDF 수정 |
| 2 | **frontend DEFAULT_ROBOT_ID hardcoded** | ★★ | frontend code |
| 3 | **React infinite loop on toggle** | ★★ | frontend code |
| 4 | zenoh stale queryable on PC restart | ★ | 운영 문제 (재시작 순서) |
| 5 | Live PC 결과 사선 + TCP 표시 위치 | (1번 fix 후 재확인) | (1번 의존) |

---

## 1. URDF joint limits 너무 좁음 (root cause of 사선)

### 증상
- 사용자가 토크오프 자리 motor 4번을 *limit 보다 더 아래로* 굽힘
- 3D viewer 의 URDF model 은 limit 까지만 표시 (urdf-loader clip)
- → URDF FK 의 tcpMatrix 가 실제와 다름
- → cameraMatrix = tcpMatrix · handEyeMatrix 도 틀림
- → Live PC 가 *사선으로 잘못된 위치* 에 박힘
- 캘이 나쁜 게 아니라 **URDF limit 가 mechanical 실제보다 좁아서 FK 자체가 틀린 것**

### 현재 URDF limits ([robot/so101_6dof/urdf/so101_6dof.urdf](robot/so101_6dof/urdf/so101_6dof.urdf))

| Joint | line | lower (rad) | upper (rad) | 범위 (deg) | 비고 |
|---|---|---|---|---|---|
| joint1 | 285 | -0.174 | +1.746 | -10°..+100° | 너무 좁음 (양쪽 비대칭) |
| joint2 | 326 | 0 | +2.094 | 0..+120° | 단방향 |
| joint3 | 317 | -2.094 | 0 | -120°..0° | 단방향 |
| **joint4** | 309 | **-1.222** | **+1.222** | **±70°** | **사용자가 초과한 joint — fix 1순위** |
| joint5 | 301 | -1.518 | +1.518 | ±87° | |
| joint6 | 293 | 0 | +3.142 | 0..180° | 단방향 |

### 의심 (확정 X)
- URDF 자리 CAD export 또는 LeRobot 의 default 값일 가능성. SO-101 6DOF mod 의 실제 mechanical range 와 안 맞음
- Feetech STS3215 자리 raw 0..4095 = ±π 범위. URDF 가 그 절반도 못 씀

### 다음 세션 작업

1. **각 joint 의 실제 mechanical range 측정** — 사용자가 robot 자세 잡으면서 motor positions raw int 의 min/max 직접 확인. 또는 motors.yaml 의 limit spec 참조
2. **URDF limit 다 수정** — 6 joint
3. backend + frontend 재시작 → URDF reload
4. Live PC 다시 확인 — 사선 사라지면 root cause 확정

**참고** — 풀어주는 정도 결정 분기:
- (a) 다 ±π 로 풀고 self-collision 은 PyBullet 가 잡게 (PyBullet 의 self-collision detection 활용)
- (b) 사용자가 robot 자세 잡으면서 실측 한계까지

---

## 2. frontend DEFAULT_ROBOT_ID hardcoded "omx_f_0"

### 증상
- 첫 page load 시 `bridge.callService` 의 default robotId = `omx_f_0` (constants/index.ts hardcoded)
- useRobots fetch 가 backend `/robots` 의 default (`so101_6dof_0`) 받기 *전* 에 호출하면 → `horibot/omx_f_0/...` 로 expand
- omx_f_0 는 disabled + rgbd capability 없음 → scene3d_node 가 omx 용 service register 안 함 → zenoh queryable 자리 없음 → timeout 또는 silently fail
- 13:20:50 PC log 의 첫 Timeout 이 이거 (이후 useRobots fetch 완료되면서 so101 로 update)

### 코드 위치
- [frontend/src/constants/index.ts:22-23](frontend/src/constants/index.ts#L22-L23):
  ```ts
  export const DEFAULT_ROBOT_ID =
    import.meta.env.VITE_DEFAULT_ROBOT_ID || "omx_f_0";
  ```
- [frontend/src/hooks/useRobots.ts:23-25](frontend/src/hooks/useRobots.ts#L23-L25): fetch 후 setDefaultRobotId(data.default) — 한 번만

### Quick fix (안 함, architectural fix 권장)
- `"omx_f_0"` → `"so101_6dof_0"` hardcoded 변경. fragile (다음에 robot 추가/변경 시 또 깨짐)

### 정석 fix (다음 세션 작업)
- bridge.callService 가 *defaultRobotId 가 backend 로부터 set 되기 전엔* reject 또는 await (initialization gate)
- 또는 defaultRobotId 를 zustand store 로 reactive 하게 — useDefaultRobotId hook 으로 wrap, 변경 시 dependent component rerender
- bootstrap timing 정리 — useRobots fetch 끝나기 전엔 service call 시도 자체 막기

---

## 3. React infinite loop on PointCloud toggle

### 증상
토글 ON 시 browser console error:
```
Maximum update depth exceeded. This can happen when a component
repeatedly calls setState inside componentWillUpdate or componentDidUpdate.
React limits the number of nested updates to prevent infinite loops.
    at setState (vanilla-DsXrSwqO.js:10:14)
    at Object.setTopicData (store.ts:32:5)
    at bootstrap.ts:63:42
    at bridge.ts:139:28
```

### 진단 chain
1. bridge WS 가 message 받음 → [bridge.ts:139](frontend/src/api/bridge.ts#L139) `_handleIncoming` → topic listener 호출
2. listener (bootstrap.ts:63) 가 `setTopicData(wire, data)` 호출
3. [store.ts:32](frontend/src/framework/store.ts#L32) `setTopicData` 가 *전체 topicData object spread* (`{...s.topicData, [k]: v}`) → reference 변경
4. **어떤 component** 가 setState 자리 매 render 자리 호출 → infinite loop
5. *어떤 component 인지 미확정* — React DevTools Profiler 또는 console 의 full component stack 필요

### 의심 원인 (확정 X)
- bootstrap 자리 *모든 topic* subscribe — high-frequency topic (MOTOR_STATE_JOINT 20Hz 등) 의 setTopicData 가 자리 자리 자리 component rerender 트리거
- `useTopic` selector 가 specific wire 만 select 라 *그 component 자체 자리 자리* 가능성 적음 — 다른 component (useFrameworkStore 의 다른 selector) 의심
- 또는 *useService* (`useFrameworkStore.serviceData[expanded]` selector) 자리 자리 자리 자리 자리 자리 — toggle 시 setServiceData 호출도 됨

### Quick fix 시도 자리
[store.ts:31-32](frontend/src/framework/store.ts#L31-L32) 자리 reference equality skip:
```ts
setTopicData: (k, v) =>
    set((s) => {
      if (s.topicData[k] === v) return s;
      return { topicData: { ...s.topicData, [k]: v } };
    }),
```
근데 backend 가 매 message 새 object 보내니까 reference equality 안 맞을 가능성 큼 — 부분 fix 만 됨

### 정석 fix (다음 세션 작업)
- React DevTools Profiler 로 어느 component 가 loop 만드는지 확정
- 그 component 의 useEffect dep / useCallback / useMemo 의 stability 점검
- *render 안에서 setState 호출* 또는 *unstable dep 가진 useEffect 에서 setState* 패턴 찾아서 fix
- (참고) CLAUDE.md 의 RobotModel.tsx:113-134 dockview leak fix (commit f15a20b) 와 비슷한 패턴 — 다른 component 에 같은 문제 잔존 가능

---

## 4. Zenoh stale queryable on PC restart (운영 issue)

### 증상
- camera Pi 가 먼저 시작 (11:42)
- PC backend 가 그 후 시작 (13:20)
- PC 가 camera Pi 의 queryable 잡음 → 동작 OK
- 사용자가 camera Pi 만 재시작 (13:26) — PC 는 그대로
- PC 가 *옛 queryable reference* 들고 있음 → service call 시 Timeout
- Color stream (pub/sub) 은 정상 (publisher 는 새 publisher 자동 pickup)
- Depth stream (service queryable) 만 fail

### 해결
- 한 쪽 재시작 시 다른 쪽도 재시작 (운영 정리)
- 또는 zenoh peer mode 의 queryable re-discovery 메커니즘 검토 (zenoh 0.x → 1.x 자리 자리 자리 자리 자리 자리)
- 또는 host_pc.yaml 의 `zenoh.connect` 에 명시적 `tcp/192.168.x.y:7447` (multicast scout 우회)

### 다음 세션 작업
- 운영 가이드에 "한 쪽 재시작 시 모두 재시작" 박기 ([scan_pipeline_readiness.md](scan_pipeline_readiness.md) update)
- 또는 zenoh re-discovery 메커니즘 코드 자리 추가

---

## 5. Live PC 결과 사선 (URDF limit fix 후 재확인)

### 증상
- URDF robot model 자리 gripper 거의 수평
- 실제 robot 자리 4번 motor 자리 더 아래로 굽힘
- Point cloud (책상 자리) 자리 사선으로 표시 — 책상 면 자리 수평 아님
- TCP 표시 위치 (3D viewer 의 빨간/초록/파랑 axis) 자리 자리 자리 자리 자리

### Root cause (의심)
- **#1 (URDF limit) fix 자리 자리** — URDF FK 자리 motor 실제 angle 자리 자리 자리 자리 cameraMatrix 자리 자리 → cloud 사선
- 캘 자체 quality 자리 자리 자리 자리 (캘 σ_R 0.801°/σ_t 7.53mm 자리 algorithmic optimum 자리 자리, [handeye_sigma_floor_so101.md](handeye_sigma_floor_so101.md))

### 다음 세션 작업
- #1 fix 후 다시 Live PC 토글 → 사선 사라지나 확인
- 사라지면 → 캘 적용 OK 확정
- 안 사라지면 → 다른 root cause (handEyeMatrix 계산 오류, RobotModel emit 오류 등) 추적

---

## 진행 순서 (다음 세션)

1. **URDF joint limits 측정 + 수정** (#1) — 가장 큰 영향, root cause
2. backend + frontend 재시작 → Live PC 다시 확인 (#5 확정)
3. defaultRobotId 정석 fix (#2) — bootstrap timing 정리
4. React infinite loop component 추적 + fix (#3) — DevTools Profiler 필요
5. Zenoh stale queryable 운영 가이드 박기 (#4)

## 진단 자리 사용한 도구

- PC backend log (스크롤 timeline)
- 모터 Pi backend log
- camera Pi backend log + `ps -eo pid,lstart,cmd`
- browser console error (Maximum update depth exceeded)
- 3D viewer 시각 (URDF 자리 cloud 자리 자리 자리)
- 코드 grep (Scene3DNode register, SceneControlsPanel, useScene3DStore, bridge.callService chain)

## 관련 문서

- [scan_pipeline_readiness.md](scan_pipeline_readiness.md) — SO-101 scan 시작 전 코드 검토
- [scan_interactive_design.md](scan_interactive_design.md) — Scan interactive workflow design (다음 세션 진입점)
- [handeye_sigma_floor_so101.md](handeye_sigma_floor_so101.md) — 캘 floor 진단
