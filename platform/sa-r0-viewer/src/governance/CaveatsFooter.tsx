import { sandboxTypography } from "@/theme/sandboxTheme";

type Props = { compareMode?: boolean };

export function CaveatsFooter({ compareMode }: Props) {
  return (
    <footer className="border-t border-slate-800/70 bg-slate-950/90 px-4 py-2.5">
      <p className={sandboxTypography.caption}>
        Read-only replay analysis only. Not operational readiness, not tactical authority, not
        live state, not HITL, not causal proof. Mirrors ≠ authority.
      </p>
      {compareMode && (
        <p className={`mt-1 ${sandboxTypography.caption} text-amber-200/70`}>
          Compare mode shows explanatory replay diffs only — not validated effectiveness or
          deployment superiority.
        </p>
      )}
    </footer>
  );
}
