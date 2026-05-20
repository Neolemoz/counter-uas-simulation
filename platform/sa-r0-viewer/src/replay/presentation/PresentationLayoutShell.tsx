import type { ReactNode } from "react";

type Props = {
  left?: ReactNode;
  center: ReactNode;
  bottom?: ReactNode;
  className?: string;
};

export function PresentationLayoutShell({ left, center, bottom, className }: Props) {
  return (
    <main className={`grid min-h-0 flex-1 grid-cols-1 gap-3 p-3 lg:grid-cols-12 ${className ?? ""}`}>
      {left ? <aside className="flex flex-col gap-3 lg:col-span-3">{left}</aside> : null}
      <div className={`flex min-h-0 flex-col gap-3 ${left ? "lg:col-span-9" : "lg:col-span-12"}`}>
        {center}
        {bottom}
      </div>
    </main>
  );
}
