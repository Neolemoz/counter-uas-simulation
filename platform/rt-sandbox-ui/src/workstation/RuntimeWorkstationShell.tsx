import type { ReactNode } from "react";

export function RuntimeWorkstationShell({
  header,
  sessionRail,
  workflowStrip,
  cognitionColumn,
  worldColumn,
  vizColumn,
  globeFooter,
  mirrorsColumn,
  pipelineFooter,
  diagnostics,
}: {
  header: ReactNode;
  sessionRail: ReactNode;
  workflowStrip: ReactNode;
  cognitionColumn?: ReactNode;
  worldColumn?: ReactNode;
  vizColumn?: ReactNode;
  globeFooter?: ReactNode;
  mirrorsColumn: ReactNode;
  pipelineFooter: ReactNode;
  diagnostics: ReactNode;
}) {
  const hasCognition = Boolean(cognitionColumn);
  const hasWorld = Boolean(worldColumn);
  const hasViz = Boolean(vizColumn);

  return (
    <main className="mx-auto max-w-7xl space-y-4 p-4">
      {header}

      <section className="space-y-3" aria-label="Session controls">
        {sessionRail}
        {workflowStrip}
      </section>

      {hasCognition || hasWorld || hasViz ? (
        <section className="space-y-2" aria-label="Globe and editing workspace">
          <div className="grid gap-4 lg:grid-cols-12">
            {hasCognition && (
              <div
                className="flex flex-col gap-4 lg:col-span-3"
                aria-label="Cognition rail"
              >
                {cognitionColumn}
              </div>
            )}
            {hasViz && (
              <div
                className={`flex flex-col gap-4 ${hasCognition ? "lg:col-span-6" : "lg:col-span-7"}`}
                aria-label="Globe and layers"
              >
                {vizColumn}
              </div>
            )}
            {hasWorld && (
              <div
                className={`flex flex-col gap-4 ${hasCognition ? "lg:col-span-3" : "lg:col-span-5"}`}
                aria-label="World editing"
              >
                {worldColumn}
              </div>
            )}
          </div>
          {globeFooter ? (
            <div aria-label="Background session summary">{globeFooter}</div>
          ) : null}
        </section>
      ) : null}

      <section aria-label="Telemetry mirrors">{mirrorsColumn}</section>

      <section aria-label="Capture and handoff">{pipelineFooter}</section>

      <section aria-label="Diagnostics">{diagnostics}</section>
    </main>
  );
}
