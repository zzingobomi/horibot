# Roadmap

진행 중/예정 작업 기록용. 이미 구현 완료된 항목은 git log + 관련 docs/ 문서로 가니까 여기는 **미래 작업**만.

> 현재 active 작업: TSDF apply 검증 (브랜치 `feat/tsdf-apply`). MeshLayer + scan capture + build_mesh 서비스가 들어와 있고, 라이브로 적용한 결과 평가 단계.

---

## TSDF / PointCloud

- (관찰 단계) Mesh 품질이 캘 σ_t 7.94mm 영향을 얼마나 받는지 — `voxel_size=2mm`, `sdf_trunc=10mm` 기본값에서 두께/이중벽 양상 확인. 영향 크면 voxel size 키우거나 ICP `icp_max_dist` 조정.
- 자세 수 vs mesh 품질 trade-off — 10자세 vs 20자세에서 fragment 개수, hole 개수 비교.
- (보류) Colored ICP — point-to-plane으로 부족할 때만. JPEG 압축이 텍스처에 노이즈 줄 수 있어 현재 미채택.
- (보류) Mesh smoothing / hole filling — Open3D `filter_smooth_taubin`, `fill_holes`. 1차 결과 보고.

### End-to-end 정확도 평가 시나리오 (집에서 실행 예정)

**목적**: "사용자가 mesh에서 클릭한 점 ↔ EE가 실제 도착한 물리 위치" 오차 = 이 시스템이 사용자에게 줄 수 있는 정확도. TSDF 단독 평가가 아니라 mesh + 캘 + IK + sag 전체 스택의 end-to-end 평가.

**셋업**:
- 20mm XYZ 캘리브레이션 큐브 1개 (MakerWorld) — 캘리퍼스로 실측해서 ground truth 갱신.
- (선택) Pointer tool 1개 — 그리퍼가 grasp하는 기둥(10×10×30mm) + 30~40mm 원뿔 끝. 하드웨어 교체 X. tool 출력 후 "grasp center → tip" 거리 캘리퍼스로 재서 TCP offset에 박음.
- 또는 단순히 그리퍼 닫고 finger 사이 중심점을 TCP로 사용 (정확도 살짝↓).

**절차**:
1. 큐브 책상 위에 놓고 TSDF scan.
2. Workspace3D에서 mesh 표시, 큐브 꼭짓점/feature 클릭.
3. 클릭 좌표로 `move_l` (Z+30~50mm hover 먼저 → 천천히 descent → 표면 1~2mm 위 정지).
4. 카메라/캘리퍼스로 needle 끝과 실제 큐브 feature 사이 X/Y/Z 오차 측정, 점별 기록.

**측정 포인트 (큐브 옮기지 않고 한 자리에서)**:
- 윗면 꼭짓점 4 + 변 중점 4 + 중심 1 = 9점 (모두 위에서 접근 가능).
- **바닥 4 꼭짓점은 버린다** — 그리퍼가 측면으로 접근 못 함.
- Z 다양화 원하면 50mm 큐브 추가 1개로 Z 두 단계 확보.

**분리 가능한 오차**:
- `클릭점 - 큐브 GT` = TSDF + hand_eye 오차 (mesh가 진실에서 얼마 떨어짐)
- `EE 도달점 - 클릭점` = IK + sag + joint/link_offset 오차 (명령한 곳에 얼마나 갔나)
- `EE 도달점 - 큐브 GT` = end-to-end 오차 (사용자 체감 숫자)
- 9점 평균 = systematic shift, 분산 = random/local error

**실행 전 선행 작업** (TSDF 1차 결과 보고 나서 착수):
- [MeshLayer.tsx](frontend/src/components/workspace3d/3d/MeshLayer.tsx)에 raycaster onClick → 좌표 추출 → "이 점으로 move_l" UI.
- Pointer tool TCP offset 처리 (사용 시).
- Safe approach 시퀀스 (hover → descent → stop above surface) 헬퍼.

**유의사항**:
- 큐브 실측해서 ground truth 갱신 안 하면 프린터 인쇄 오차(elephant foot 등 ±0.1~0.3mm)가 TSDF 오차로 잘못 잡힘.
- 한 자리 한 자세 평가 — 작업공간 전체 정확도는 별도. 첫 숫자 보고 나서 다중 위치 평가 여부 결정.

## Calibration

- 현재 σ_rot 0.65° / σ_t 7.94mm 달성. 더 내리려면:
  - D405 마운트 강성 — XL330 wrist 그룹 끝에 카메라 매달려있는 구조라 작은 sag 잔존 가능.
  - Joint encoder 분해능 — XL430의 4096 분해능이 본질적 floor.
  - 두 가지 다 H/W 변경 필요 → 소프트 측면에서는 사실상 saturate 상태로 보고 TSDF 결과로 검증.

## 분산 운영

- (보류) 모터 Pi의 motion + motor 통합 latency 측정 — 현재 100Hz `MOTOR_CMD_JOINT` 큐 모니터링은 없음. 명령 누락 시 trajectory가 끊겨야 정상.
- (보류) Zenoh peer 발견 실패 케이스 — 멀티캐스트 차단 환경 디버깅 절차 정리.
