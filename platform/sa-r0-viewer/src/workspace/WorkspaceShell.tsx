import type { ReactNode } from "react";
import { cn } from "@/lib/utils";
import type { WorkspaceLayoutVariant } from "./types";

type Props = {
  t3?: ReactNode;
  t1: ReactNode;
  t2?: ReactNode;
  t4t5?: ReactNode;
  t6?: ReactNode;
  layoutVariant?: WorkspaceLayoutVariant;
  className?: string;
};

const CENTER_SPAN: Record<WorkspaceLayoutVariant, string> = {
  segment: "lg:col-span-6",
  compare: "lg:col-span-6",
  filmstrip: "lg:col-span-9",
  publication: "lg:col-span-9",
  focus: "lg:col-span-12",
};

export function WorkspaceShell({
  t3,
  t1,
  t2,
  t4t5,
  t6,
  layoutVariant = "segment",
  className,
}: Props) {
  const hasLeft = Boolean(t3) && layoutVariant !== "focus";
  const hasRight = Boolean(t4t5) && layoutVariant !== "publication" && layoutVariant !== "focus";
  const centerSpan =
    layoutVariant === "publication" && !hasLeft
      ? "lg:col-span-12"
      : layoutVariant === "publication" && hasLeft
        ? "lg:col-span-9"
        : CENTER_SPAN[layoutVariant];

  return (
    <main
      className={cn(
        "grid min-h-0 flex-1 grid-cols-1 gap-4 p-4 lg:grid-cols-12",
        layoutVariant === "publication" && "sandbox-publication",
        className,
      )}
    >
      {hasLeft ? (
        <aside className="flex flex-col gap-4 lg:col-span-3">{t3}</aside>
      ) : null}
      <div className={cn("flex min-h-0 flex-col gap-4", centerSpan)}>
        <div className="flex min-h-0 flex-1 flex-col gap-4">{t1}</div>
        {t2}
        {t6}
      </div>
      {hasRight ? (
        <aside className="flex flex-col gap-4 lg:col-span-3">{t4t5}</aside>
      ) : null}
    </main>
  );
}
