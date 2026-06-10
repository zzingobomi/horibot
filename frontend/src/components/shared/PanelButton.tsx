/**
 * Panel 공용 버튼 — RobotState/Calibration 패널과 같은 zinc 다크 톤.
 *
 * shadcn `<Button>` 은 light theme CSS var 가 새어나와 다크 패널 안에서 흰
 * 카드/흰 배경으로 풀려 통일성을 깸. 본 컴포넌트는 zinc-* 클래스 명시.
 */
import { forwardRef } from "react";

type Variant = "primary" | "secondary" | "outline" | "ghost" | "danger";

const VARIANTS: Record<Variant, string> = {
  primary:
    "bg-emerald-500/15 hover:bg-emerald-500/25 text-emerald-300 border border-emerald-500/30 disabled:bg-zinc-800/40 disabled:text-zinc-600 disabled:border-zinc-800/60",
  secondary:
    "bg-zinc-800/60 hover:bg-zinc-700/60 text-zinc-200 border border-zinc-700/60 disabled:bg-zinc-900/40 disabled:text-zinc-600 disabled:border-zinc-800/60",
  outline:
    "bg-transparent hover:bg-zinc-800/60 text-zinc-300 border border-zinc-700/60 hover:text-zinc-100 disabled:text-zinc-600 disabled:border-zinc-800/60",
  ghost:
    "bg-transparent hover:bg-zinc-800/60 text-zinc-400 hover:text-zinc-100 border border-transparent disabled:text-zinc-600",
  danger:
    "bg-red-500/15 hover:bg-red-500/25 text-red-300 border border-red-500/30 disabled:bg-zinc-800/40 disabled:text-zinc-600 disabled:border-zinc-800/60",
};

interface PanelButtonProps
  extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: Variant;
}

export const PanelButton = forwardRef<HTMLButtonElement, PanelButtonProps>(
  ({ variant = "secondary", className = "", ...props }, ref) => (
    <button
      ref={ref}
      className={`text-[11px] font-mono uppercase tracking-wide px-2.5 py-1.5 rounded transition-colors disabled:cursor-not-allowed ${VARIANTS[variant]} ${className}`}
      {...props}
    />
  ),
);
PanelButton.displayName = "PanelButton";
