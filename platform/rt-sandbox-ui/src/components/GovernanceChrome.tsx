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
}: {
  title: string;
  children: React.ReactNode;
  className?: string;
}) {
  return (
    <section
      className={clsx(
        "rounded-lg border border-slate-700 bg-slate-900/80 p-4",
        className,
      )}
    >
      <h2 className="mb-3 text-sm font-semibold uppercase tracking-wide text-slate-300">
        {title}
      </h2>
      {children}
    </section>
  );
}
