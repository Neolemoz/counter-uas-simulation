import type { ReactNode } from "react";
import { WorkspaceShell } from "@/workspace/WorkspaceShell";

type Props = {
  left?: ReactNode;
  center: ReactNode;
  bottom?: ReactNode;
  className?: string;
};

/** Publication layout — delegates to WorkspaceShell (PLAN-SA-H1 Concept C). */
export function PresentationLayoutShell({ left, center, bottom, className }: Props) {
  return (
    <WorkspaceShell
      layoutVariant="publication"
      className={className}
      t3={left}
      t1={center}
      t2={bottom}
    />
  );
}
