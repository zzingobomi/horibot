/**
 * 씬 시각화 **의미 색** SSOT — hex 를 공유하는 모듈이 아니라 *시각적 의미*를
 * 공유하는 모듈. 새 시각화는 hex 를 고르지 말고 여기서 **역할**을 고른다.
 * ([docs/frontend.md] 색 시스템)
 *
 * 지금 소비자 있는 역할만 토큰화 — warning/constraint(red/orange), box ghost,
 * trajectory 등 미래 역할은 그 소비자가 생길 때 추가 (소비자 없는 상수 선제작 X).
 * 전체 팔레트(미래 포함)는 문서에 박음.
 *
 * ⚠️ AxisFrame 의 X/Y/Z = red/green/blue 는 이 체계가 **아니다** — 그건 좌표축
 * 관례(RGB=XYZ)지 의미 색이 아니라서 여기 넣지 않는다. 두 체계는 완전히 별개:
 *   축 색   : X=red / Y=green / Z=blue  (기하 관례)
 *   의미 색 : 아래 토큰                  (역할 구분)
 */
export const VizColor = {
  /** command preview / ghost / simulation — 가상·예측 표현 (waypoint ghost) */
  PREVIEW: "#8b5cf6",
  /** camera frustum / 센서 계열 (D405 frustum) */
  SENSOR: "#66ccff",
  /** 인식 결과 / attention (task 검출 마커) */
  DETECTION: "#34d399",
  /** 작업 목표 — "여기로 가야 한다" (grasp/place 목표점). TCP 와 구분: 목표 ≠ 기준점 */
  TARGET: "#f59e0b",
  /** 후보 / 비활성 (검출 후보) */
  CANDIDATE: "#71717a",
  /** 경고 / 제약 위반 — "여기서 막힘·도달 불가" (preview 경로 실패 지점) */
  WARNING: "#ef4444",
  /** 로봇 기준 프레임 — "현재 손 끝이 어디인가" (TCP frame label). 항상 존재하는 기준점 */
  TCP: "#ffcc44",
  // 실물(real world object)은 tint 없음 = RobotModel 원본 material (gray/white).
} as const;
