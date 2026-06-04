# Slice A / B / C 검증 가이드

[multi_robot_phase2_frontend.md](multi_robot_phase2_frontend.md) §6 의 구현 결과를
실 hardware + dev 서버에서 검증하는 순차 절차.

**전제**: 코드 정합성 (ruff / pyright / tsc / lint) 은 통과한 상태. 본 문서는
*실 동작* 검증 + fine-tune 발견 자리 체크.

## 0. 사전 준비

```powershell
# Backend deps sync (psutil 새로 추가됨)
cd backend
uv sync

# Frontend deps (변화 없음, 혹시 모를 lock drift 대비)
cd ..\frontend
pnpm install
```

OpenRB-150 USB 연결 / D405 카메라 연결 확인.

## 1. Backend 부팅 검증

```powershell
cd backend
uv run python main.py
```

기대 로그:
- `[INFO] RobotRegistry load 완료: 2 robot — ['omx_f_0', 'so101_0']`
- `[INFO] default robot_id=omx_f_0 (N=1)` ← enabled=true 가 omx_f_0 만이라 default 잡힘
- `[INFO] Zenoh 구독 설정 완료 (robots=['omx_f_0', 'so101_0'])`
- 노드별 `시작됨` 메시지 (motor / camera / motion / calibration / task / detector / pointcloud)
- 브릿지 `시작: ws://0.0.0.0:8000`

**FAIL 자리**:
- `RobotRegistry load` 실패 → robots.yaml 의 base_pose 형식 / so101_0 entry 확인
- `default()` RuntimeError → enabled=true robot 이 0 또는 2개 이상. robots.yaml 확인
- `Zenoh 구독 설정` 누락 robot → bridge.py 의 robots.yaml enumerate 자리 확인
- 모터 연결 실패 → COM 포트 / instance.yaml. enabled=true 인 omx_f_0 의 instance.yaml 만 영향

## 2. 빠른 API 점검 (curl / 브라우저)

새 터미널:
```powershell
curl http://localhost:8000/robots
curl http://localhost:8000/tasks
curl http://localhost:8000/system
```

기대:
- `/robots` → `{"robots": [{"id":"omx_f_0",...}, {"id":"so101_0",...}], "default":"omx_f_0"}`
- `/tasks` → `{"tasks": ["pick_and_place"]}`
- `/system` → `{"cpu_pct":..., "mem_used_mb":..., "zenoh_peers":..., ...}`

## 3. Frontend dev 서버

```powershell
cd frontend
pnpm dev
```

브라우저 `http://localhost:5173`.

### 3-1. Dashboard (`/`)
체크:
- [ ] **Robots Online** 카드에 `omx_f_0` `so101_0` 2 entry. `omx_f_0` = OK (motor_node heartbeat), `so101_0` = Viz-only.
- [ ] count: `1 / 1` (enabled 만 카운트)
- [ ] **System** 카드: Bridge OK, Camera OK (D405 연결됐다면), Zenoh peers / CPU / Mem 숫자 표시 (5초마다 갱신)
- [ ] robot id 클릭 → `/robots/<id>` 이동

**FAIL 자리**:
- Robots 카드 빈 채 — `/robots` fetch 실패. 브라우저 콘솔에서 CORS / 404 확인
- "No Heartbeat" — backend 부팅됐으나 motor_node 시작 실패. backend 로그 확인
- System 카드 `—` — `/system` 응답 실패. backend 로그
- CPU / Mem 가 5초 갱신 안 됨 — useSystemMetrics 의 setInterval 확인

### 3-2. Workspace `/robots/omx_f_0` (focus mode)
체크:
- [ ] 두 URDF 보임. `omx_f_0` 가 origin, `so101_0` 가 x=+0.4m 위치
- [ ] `omx_f_0` 가 불투명, `so101_0` 가 흐릿함 (dim opacity 0.25)
- [ ] 4개 panel mount (Robot State / Motion / Scene Controls / Calibration / Calibration Actions / Point Cloud)
- [ ] Motion panel 의 Joint/J/L/C/P/TCP Tabs 동작 — 모터에 명령 들어감
- [ ] Calibration Actions 의 Intrinsic / Hand-Eye Tabs 동작
- [ ] 우상단 메타: id / type / enabled
- [ ] 카메라 OrbitControls 가 omx_f_0 의 base 위치 lookAt

**FAIL 자리**:
- URDF 한 벌만 보임 → RobotLayer / useRobots / `/robots` 응답 확인
- 두 URDF 가 같은 위치 (겹침) → robots.yaml 의 base_pose / RobotModel 의 basePose prop
- dim 안 됨 / 너무 짙음 → RobotLayer 의 `dimOpacity={0.25}` 조정 (RobotLayer.tsx 또는 props)
- panel 6개 화면 넘침 / overflow → RobotsPage 의 PANELS 배열 width/height 조정
- Motion panel 너무 작아 Tabs 안 보임 → `motion` entry 의 width/height 늘림

### 3-3. `/robots/so101_0` (viz-only focus)
체크:
- [ ] so101_0 가 불투명, omx_f_0 가 흐릿
- [ ] 메타: enabled=false (Viz-only)
- [ ] motor 명령 (Motion panel) 가능은 함 — 다만 자동 expand 가 `so101_0` 로 expand 해 publish — backend 에 so101_0 의 motor 노드 없으니 무응답
- [ ] joint state 안 들어옴 → URDF home pose

**의도된 자리** — 가짜 robot 이라 실 명령 작동 X. URDF 보이는 것 자체가 검증.

### 3-4. World (`/world`)
체크:
- [ ] 두 URDF 모두 불투명
- [ ] OrbitControls 가 두 robot 중심을 잡음
- [ ] 카메라 default position `[0.7, 0.6, 0.7]` — 약간 멀리서 봄
- [ ] panel 없음

### 3-5. Tasks (`/tasks/pick_and_place`)
체크:
- [ ] focus=null (World view)
- [ ] panel 4개 mount (Prompt / Task Progress / Camera Feed / Robot State)
- [ ] Prompt 에 "큐브를 박스에" 같은 문장 → Run
- [ ] Task Progress 가 step tree 표시 + 진행 상황
- [ ] Camera Feed = default robot 의 카메라 영상

**FAIL 자리**:
- 라우트 진입 시 "task 없음" 표시 → `/tasks` endpoint 응답 확인
- Sidebar 의 Tasks 섹션이 비어있음 → useTasks fetch / `/tasks` 응답

### 3-6. Sidebar / Navigation
체크:
- [ ] **Robots** 섹션: omx_f_0 (밝게) + so101_0 (흐릿하게 — enabled=false)
- [ ] **Tasks** 섹션: pick_and_place 1 entry
- [ ] 클릭 → 각 라우트 이동
- [ ] 사이드바 collapse / expand 동작 (localStorage `omx.sidebar.collapsed`)

## 4. 카메라 stream 검증

URL 직접: `http://localhost:8000/robots/omx_f_0/camera/stream` → MJPEG 영상.

체크:
- [ ] D405 영상 (HW 연결 시)
- [ ] `/robots/so101_0/camera/stream` 도 endpoint 는 200 응답하지만 영상 없음 (publisher 없으니)

## 5. 발견할 자리 (fine-tune 후보) 체크리스트

만져보고 *불편한* 자리:
- [ ] RobotLayer `dimOpacity` — 0.25 가 적당? 너무 흐릿하면 0.4
- [ ] WorldPage 카메라 default `[0.7, 0.6, 0.7]` — 두 robot 잘 보이는지
- [ ] RobotsPage 의 6개 panel default layout — 화면 잘리는지
- [ ] MotionPanel width 320 / height 360 — Tabs / 컨트롤 잘 들어가는지
- [ ] CalibrationActionsPanel 도 동일
- [ ] so101_0 의 base_pose.x = 0.4 — 두 robot 이 적당히 떨어졌는지
- [ ] focus 전환 시 카메라 transition — 부드러운지 (현재는 OrbitControls target 즉시 변경, key remount 로 layout 새로 load)

## 6. 깨진 자리 발견 시

- **로그 위치**:
  - backend: stdout (직접 보임)
  - frontend: 브라우저 DevTools Console + Network
- **자주 보는 자리**:
  - WebSocket `/ws` connect 실패 → bridge 미실행 또는 CORS
  - `/robots` 404 → backend 의 zenoh_bridge.py 라우트 등록 확인
  - `Failed to load URDF` → BASE_URL / robotType 경로. RobotModel 의 `loader.workingPath` 확인
  - panel mount 안 됨 → panelComponents.ts 의 key 일치 확인

## 7. 검증 통과 후 다음 단계

- Slice C reversible 진입 가능 (Layer registry / Page Preset / store dict 화 등)
- 발견된 fine-tune 자리 fix
- robot-to-robot extrinsic 캘리브레이션 design (so101 실 hardware 도착 시)
- `/system` 의 CPU/Mem 메트릭을 Dashboard 그래프로 (필요해지면)
- Settings 페이지 / ConnectionStatus 의 robot 별 노드 표시 (현재 last-wins)
