/** Calm research sandbox visual tokens (PLAT-SA-H5). Tailwind class strings only. */

export const sandboxSurfaces = {
  panel: "sandbox-panel rounded-lg border border-slate-700/80 bg-slate-900/70",
  panelInset: "rounded-md border border-slate-800/90 bg-slate-950/60",
  rail: "flex flex-col gap-3",
  publicationBar:
    "rounded-lg border border-violet-900/35 bg-violet-950/15 px-3 py-2.5 shadow-sm",
  storyCard:
    "w-full rounded-lg border border-slate-700/70 bg-slate-900/60 px-3 py-2.5 text-left transition-colors hover:border-violet-800/50 hover:bg-slate-800/50",
} as const;

export const sandboxTypography = {
  pageTitle: "text-lg font-semibold tracking-tight text-slate-100",
  sectionLabel: "text-xs font-semibold uppercase tracking-wide text-slate-400",
  body: "text-sm leading-relaxed text-slate-300",
  caption: "text-[11px] leading-snug text-slate-500",
  monoRef: "font-mono text-[10px] text-slate-500",
  chapterSummary: "text-sm leading-relaxed text-violet-100/90",
} as const;

export const sandboxButtons = {
  subtle:
    "rounded-md border border-slate-600/80 bg-slate-800/80 px-2.5 py-1 text-xs text-slate-200 transition-colors hover:border-slate-500 hover:bg-slate-700/80 disabled:opacity-40",
  primary:
    "sandbox-btn-primary rounded-md border border-violet-800/50 bg-violet-950/40 px-2.5 py-1 text-xs text-violet-100 transition-colors hover:bg-violet-900/40 disabled:opacity-40",
  compare:
    "rounded-md border border-amber-800/40 bg-amber-950/30 px-2.5 py-1 text-xs text-amber-100/90 hover:bg-amber-900/30",
} as const;

export const sandboxAccents = {
  presentation: "text-violet-300/90",
  compare: "text-amber-200/90",
  link: "text-cyan-300/90 hover:text-cyan-200",
} as const;

export const tierBorderClass = {
  t3: "border-slate-700/80",
  t4: "border-violet-900/35",
  t5: "border-slate-800/80",
} as const;
