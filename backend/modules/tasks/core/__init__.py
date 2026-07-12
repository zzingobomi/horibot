"""tasks/core — task 모듈들이 공유하는 프레임워크 부품 (모듈 아님 — contract.py 없음).

층위: backend/framework/ (도메인 무지 — 전송/직렬화/디스패치) 위의 **도메인 인지
프레임워크**. RobotHandle 이 motion/detector/motor 계약을 import 해야 하므로
framework/ 에 못 들어감 (역방향 의존 금지) — task 모듈들 옆에 라이브러리로 둔다.

책임 분리 (2026-07-12 설계 수렴):
  - TaskRunner  (runner.py)  — 실행 생명주기만. robot 은 id 문자열만 앎.
  - TaskContext (context.py) — 도메인 접근 (robot spec / primitive / escape hatch).
  - wire.py                  — STATE/TRACE/STEP_RESULT payload 규약 (공용 UI 가 소비).
  - metadata.py              — GET /tasks 노출용 TaskMetadata registry.
task 모듈 = 평범한 모듈 (상속/자동배선 없음): 서비스 핸들러를 명시로 쓰고 runner 에
위임. 시나리오 규칙은 둘뿐 — "ctx 받는 async 함수" + "실패는 raise".
"""
