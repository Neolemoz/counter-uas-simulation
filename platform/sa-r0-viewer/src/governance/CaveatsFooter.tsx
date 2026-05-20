type Props = { compareMode?: boolean };

export function CaveatsFooter({ compareMode }: Props) {
  return (
    <footer className="border-t border-slate-700 bg-slate-900 px-4 py-2 text-xs text-slate-500">
      Read-only replay analysis only. Not operational readiness, not tactical authority, not
      live state, not HITL, not causal proof. Mirrors ≠ authority.
      {compareMode && (
        <span className="block mt-1 text-amber-200/70">
          Compare mode shows explanatory replay diffs only — not validated effectiveness or
          deployment superiority.
        </span>
      )}
    </footer>
  );
}
