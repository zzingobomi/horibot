// jest-dom matcher(toBeDisabled / toBeInTheDocument 등) 타입을 vitest 의 expect
// (Assertion) 에 병합한다.
//
// 왜 여기(src/):  runtime 등록은 vitest.setup.ts(setupFiles)가 하지만, 그 파일은
// tsconfig.app.json 의 include("src") 밖(root)이라 tsc -b 가 augmentation 을 못 본다
// → test 파일에서 toBeDisabled 가 "Property does not exist" 로 뜸(`pnpm build` 실패).
// src/ 안 .d.ts 로 두면 tsc 가 컴파일에 포함 → augmentation 이 프로그램 전역에 적용.
// (관심사 분리: setupFiles = 런타임 matcher 등록, 이 선언 = 컴파일 타입.)
import "@testing-library/jest-dom/vitest";
