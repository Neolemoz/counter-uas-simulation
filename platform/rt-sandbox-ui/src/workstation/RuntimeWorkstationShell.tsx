import type { ReactNode } from "react";

export function RuntimeWorkstationShell({
  header,
  sessionRail,
  workflowStrip,
  worldColumn,
  vizColumn,
  mirrorsColumn,
  pipelineFooter,
  diagnostics,
}: {
  header: ReactNode;
  sessionRail: ReactNode;
  workflowStrip: ReactNode;
  worldColumn?: ReactNode;
  vizColumn?: ReactNode;
  mirrorsColumn: ReactNode;
  pipelineFooter: ReactNode;
  diagnostics: ReactNode;
}) {
  const hasWorld = Boolean(worldColumn);
  const hasViz = Boolean(vizColumn);

  return (
    <main className="mx-auto max-w-7xl space-y-4 p-4">
      {header}

      <section className="space-y-3" aria-label="Session controls">
        {sessionRail}
        {workflowStrip}
      </section>

      {hasWorld || hasViz ? (
        <div className="grid gap-4 lg:grid-cols-12">
          {hasWorld && (
            <div className="flex flex-col gap-4 lg:col-span-5">{worldColumn}</div>
          )}
          {hasViz && (
            <div className="flex flex-col gap-4 lg:col-span-7">{vizColumn}</div>
          )}
        </div>
      ) : null}

      <section aria-label="Telemetry mirrors">{mirrorsColumn}</section>

      <section aria-label="Capture and handoff">{pipelineFooter}</section>

      <section aria-label="Diagnostics">{diagnostics}</section>
    </main>
  );
}
