# API generated types

`backend/core/transport/messages/*.py` 의 Pydantic 모델로부터 자동 생성된 TypeScript type
[multi_robot_architecture.md §7.3](../../../../docs/multi_robot_architecture.md) 참조.

## 생성 방법

bridge 가 동작 중일 때 (`uv run python main.py` 에서 FastAPI 가 떠있는 상태):

```bash
cd frontend
pnpm gen:types
```

`http://localhost:8000/openapi.json` → `src/api/generated/types.ts` 로 생성.

## 사용

```ts
import type { paths, components } from "@/api/generated/types";

// service request / response type
type MoveLRequest = components["schemas"]["MoveLRequest"];
type MoveResponse = components["schemas"]["MoveResponse"];
```

## drift 방지

- 이 디렉토리의 `types.ts` 는 **git 추적** (CI 가 띄울 backend 없이도 frontend
  type check 가능해야).
- backend Pydantic 모델 변경 후 `pnpm gen:types` 안 돌리면 drift — pre-commit
  hook 또는 CI check 로 강제 (TODO).
