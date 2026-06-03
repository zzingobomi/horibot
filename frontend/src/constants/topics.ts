/**
 * 호환 re-export shim. 새 코드는 `@/api/generated/contract` 직접 import 권장.
 *
 * 본래 토픽 / 서비스 키는 backend `api_contract.py` 가 single source of truth.
 * `pnpm gen:types` 가 `/openapi.json::x-contract` 를 읽어 generated/contract.ts
 * 를 만들고, 본 파일은 기존 import 자리 (`@/constants/topics`) 보존용.
 */
export {
  Topic,
  ServiceKey,
  BINARY_TOPICS,
  type TopicKey,
  type ServiceKeyValue,
  type TopicPayloadMap,
  type ServiceMap,
} from "@/api/generated/contract";
