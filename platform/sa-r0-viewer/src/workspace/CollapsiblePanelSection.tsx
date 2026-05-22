import { useState, type ReactNode } from "react";
import { cn } from "@/lib/utils";
import { tierBorderClass } from "@/theme/sandboxTheme";

type Props = {
  id: string;
  title: string;
  tier?: "t3" | "t4" | "t5";
  defaultCollapsed?: boolean;
  helper?: string;
  children: ReactNode;
  className?: string;
};

export function CollapsiblePanelSection({
  id,
  title,
  tier = "t3",
  defaultCollapsed = false,
  helper,
  children,
  className,
}: Props) {
  const [open, setOpen] = useState(!defaultCollapsed);

  return (
    <section
      data-panel-id={id}
      className={cn(
        "sandbox-panel overflow-hidden text-sm shadow-sm",
        tierBorderClass[tier],
        className,
      )}
    >
      <button
        type="button"
        className="flex w-full items-center justify-between gap-2 px-3 py-2.5 text-left transition-colors hover:bg-slate-800/30"
        aria-expanded={open}
        onClick={() => setOpen((v) => !v)}
      >
        <span className="text-xs font-semibold uppercase tracking-wide text-slate-300">
          {title}
        </span>
        <span className="text-slate-500 tabular-nums" aria-hidden>
          {open ? "−" : "+"}
        </span>
      </button>
      {helper && open && (
        <p className="sandbox-caption border-t border-slate-800/60 px-3 pb-1.5 pt-1">
          {helper}
        </p>
      )}
      {open && (
        <div className="border-t border-slate-800/60 px-3 pb-3 pt-2">{children}</div>
      )}
    </section>
  );
}
