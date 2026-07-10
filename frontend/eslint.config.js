import js from "@eslint/js";
import globals from "globals";
import reactHooks from "eslint-plugin-react-hooks";
import reactRefresh from "eslint-plugin-react-refresh";
import tseslint from "typescript-eslint";

export default tseslint.config(
  // src/api/generated 는 gen_contract.py 가 emit 하는 생성물 — generator 가 SSOT,
  // lint 대상 아님 (empty-interface 등은 generator 출력 스타일이지 코드 문제 X).
  { ignores: ["dist", "node_modules", ".tmp", "src/api/generated"] },
  {
    extends: [js.configs.recommended, ...tseslint.configs.recommended],
    files: ["**/*.{ts,tsx}"],
    languageOptions: {
      ecmaVersion: 2023,
      globals: globals.browser,
    },
    plugins: {
      "react-hooks": reactHooks,
      "react-refresh": reactRefresh,
    },
    rules: {
      ...reactHooks.configs.recommended.rules,
      "react-refresh/only-export-components": [
        "warn",
        { allowConstantExport: true },
      ],
    },
  },
  // shadcn ui primitive 는 CLI 가 emit·regenerate 하는 vendored 디자인 시스템 —
  // cva variants(buttonVariants 등)를 컴포넌트와 같은 파일에 export 하는 게 표준
  // 출력이라 fast-refresh 규칙 제외 (src/api/generated 와 동일 성격).
  {
    files: ["src/components/ui/**"],
    rules: { "react-refresh/only-export-components": "off" },
  },
);
