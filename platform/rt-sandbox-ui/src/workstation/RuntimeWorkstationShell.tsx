import type { ComponentType, ReactNode } from "react";
import { Activity, FlaskConical, Handshake } from "lucide-react";

function SupportSection({
  title,
  children,
  icon: Icon,
}: {
  title: string;
  children: ReactNode;
  icon: ComponentType<{ className?: string }>;
}) {
  return (
    <details className="rounded-lg border border-slate-800/80 bg-slate-950/40">
      <summary className="flex cursor-pointer items-center gap-2 px-3 py-2 text-sm font-semibold text-slate-300 hover:bg-slate-900/70">
        <Icon className="h-4 w-4 text-slate-400" />
        <span>{title}</span>
      </summary>
      <div className="border-t border-slate-800/80 p-3">{children}</div>
    </details>
  );
}

export function RuntimeWorkstationShell({
  header,
  sessionRail,
  workflowStrip,
  cognitionColumn,
  tacticalColumn,
  vizColumn,
  worldColumn,
  globeFooter,
  captureFooter,
  experimentFooter,
  diagnostics,
}: {
  header: ReactNode;
  sessionRail: ReactNode;
  workflowStrip: ReactNode;
  cognitionColumn?: ReactNode;
  tacticalColumn?: ReactNode;
  vizColumn?: ReactNode;
  worldColumn?: ReactNode;
  globeFooter?: ReactNode;
  captureFooter: ReactNode;
  experimentFooter: ReactNode;
  diagnostics: ReactNode;
}) {
  const hasCognition = Boolean(cognitionColumn);
  const hasTactical = Boolean(tacticalColumn);
  const hasWorld = Boolean(worldColumn);
  const hasViz = Boolean(vizColumn);

  return (
    <main className="mx-auto max-w-[1600px] space-y-5 p-3 sm:p-4">
      <section
        className="flex flex-wrap items-center gap-3 rounded-lg border border-slate-700 bg-slate-900/60 p-3 sm:p-4"
        aria-label="Session and primary controls"
      >
        <div className="min-w-0 flex-1 basis-64">{header}</div>
        <div className="flex min-w-0 flex-[2_1_480px] flex-wrap items-center gap-2 sm:gap-3">
          {sessionRail}
        </div>
      </section>

      <section aria-label="Runtime workflow">{workflowStrip}</section>

      {hasCognition || hasWorld || hasViz ? (
        <section className="space-y-3" aria-label="Globe and editing workspace">
          <div className="grid items-start gap-4 md:gap-5 lg:grid-cols-[minmax(12rem,17rem)_minmax(0,1fr)_minmax(16rem,26rem)]">
            {(hasCognition || hasTactical) && (
              <div
                className="flex min-w-0 flex-col gap-3 lg:max-w-[18rem] xl:sticky xl:top-4"
                aria-label="Runtime cognition and tactical sandbox"
              >
                {cognitionColumn}
                {tacticalColumn}
              </div>
            )}
            {hasViz && (
              <div className="flex min-w-0 flex-col gap-3" aria-label="Cesium runtime view">
                {vizColumn}
              </div>
            )}
            {hasWorld && (
              <div
                className="flex min-w-0 flex-col gap-4 lg:max-w-[26rem] lg:justify-self-end"
                aria-label="Entity palette and editor"
              >
                {worldColumn}
              </div>
            )}
          </div>
          {globeFooter ? (
            <div
              className="rounded border border-slate-800/80 bg-slate-950/30 p-2"
              aria-label="Background session summary"
            >
              {globeFooter}
            </div>
          ) : null}
        </section>
      ) : null}

      <section className="space-y-3" aria-label="Collapsible support sections">
        <SupportSection title="Capture & handoff pipeline" icon={Handshake}>
          {captureFooter}
        </SupportSection>
        <SupportSection title="Experiment workbench" icon={FlaskConical}>
          {experimentFooter}
        </SupportSection>
        <SupportSection title="Diagnostics / health" icon={Activity}>
          {diagnostics}
        </SupportSection>
      </section>
    </main>
  );
}
