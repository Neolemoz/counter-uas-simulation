import type { ComponentType } from "react";
import { clsx } from "clsx";
import { bannersForSession } from "@/governance/banners";

export function GovernanceChrome({
  connected = false,
  multiSession = false,
}: {
  connected?: boolean;
  multiSession?: boolean;
}) {
  const banners = bannersForSession(connected, multiSession);
  return (
    <header className="border-b border-amber-700/50 bg-amber-950/50">
      <div
        role="status"
        aria-label="RT sandbox governance"
        className="mx-auto max-w-7xl px-4 py-3"
      >
        <div className="flex flex-wrap items-center justify-center gap-x-3 gap-y-2">
          {banners.map((banner, i) => (
            <span
              key={banner}
              className={clsx(
                "text-center text-xs font-semibold uppercase tracking-wide text-amber-100",
                i > 0 && "border-l border-amber-700/40 pl-3 max-sm:border-l-0 max-sm:pl-0",
              )}
            >
              {banner}
            </span>
          ))}
        </div>
      </div>
    </header>
  );
}

export function PanelShell({
  title,
  children,
  className,
  icon: Icon,
  variant = "secondary",
}: {
  title: string;
  children: React.ReactNode;
  className?: string;
  icon?: ComponentType<{ className?: string }>;
  variant?: "primary" | "secondary" | "tertiary";
}) {
  return (
    <section
      className={clsx(
        "rounded-lg bg-slate-900/75",
        variant === "primary"
          ? "border border-cyan-500/40 p-5"
          : variant === "tertiary"
            ? "border border-slate-800/70 bg-slate-950/35 p-3"
            : "border border-slate-700/60 p-4",
        className,
      )}
    >
      <h2
        className={clsx(
          "mb-3 flex items-center gap-2 font-semibold uppercase tracking-wide sm:mb-4",
          variant === "primary" ? "text-base text-cyan-50" : "text-sm text-slate-300",
        )}
      >
        {Icon && <Icon className="h-4 w-4 text-cyan-300" />}
        <span>{title}</span>
      </h2>
      {children}
    </section>
  );
}
