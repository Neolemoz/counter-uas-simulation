import { clsx } from "clsx";

const TONE_CLASS = {
  ok: "bg-emerald-900/60 text-emerald-200 border-emerald-700",
  warn: "bg-amber-900/60 text-amber-200 border-amber-700",
  error: "bg-red-900/60 text-red-200 border-red-700",
  neutral: "bg-slate-800 text-slate-300 border-slate-600",
} as const;

export type StatusBadgeTone = keyof typeof TONE_CLASS;

export function StatusBadge({
  label,
  tone = "neutral",
  title,
}: {
  label: string;
  tone?: StatusBadgeTone;
  title?: string;
}) {
  return (
    <span
      title={title}
      className={clsx(
        "inline-flex rounded border px-2 py-0.5 text-xs font-medium",
        TONE_CLASS[tone],
      )}
    >
      {label}
    </span>
  );
}
