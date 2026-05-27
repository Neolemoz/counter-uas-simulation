import { PanelShell } from "./GovernanceChrome";
import { TelemetryCognitionStrip } from "./TelemetryCognitionStrip";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";

function Field({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex justify-between gap-4 text-sm">
      <span className="text-slate-400">{label}</span>
      <span className="font-mono text-slate-100">{value}</span>
    </div>
  );
}

export function SessionLifecyclePanel({
  snapshot,
  hideCognition = false,
}: {
  snapshot: ChannelSnapshot | undefined;
  hideCognition?: boolean;
}) {
  const payload = snapshot?.payload ?? {};
  return (
    <PanelShell title="Session lifecycle">
      <div className="space-y-2">
        <Field label="state" value={String(payload.state ?? "—")} />
        <Field
          label="previous_state"
          value={String(payload.previous_state ?? "—")}
        />
        <Field
          label="command_type"
          value={String(payload.command_type ?? "—")}
        />
      </div>
      {!hideCognition && <TelemetryCognitionStrip snapshot={snapshot} />}
    </PanelShell>
  );
}

export function SessionHealthPanel({
  snapshot,
  hideCognition = false,
}: {
  snapshot: ChannelSnapshot | undefined;
  hideCognition?: boolean;
}) {
  const payload = snapshot?.payload ?? {};
  const adapterHealth =
    payload.adapter_health && typeof payload.adapter_health === "object"
      ? (payload.adapter_health as Record<string, unknown>)
      : null;

  return (
    <PanelShell title="Session health">
      <div className="space-y-2">
        <Field label="state" value={String(payload.state ?? "—")} />
        <Field
          label="runtime_mode"
          value={String(payload.runtime_mode ?? "—")}
        />
        {adapterHealth && (
          <>
            <Field
              label="adapter_alive"
              value={String(adapterHealth.alive ?? "—")}
            />
            <Field
              label="adapter_mode"
              value={String(adapterHealth.mode ?? "—")}
            />
          </>
        )}
      </div>
      {!hideCognition && <TelemetryCognitionStrip snapshot={snapshot} />}
    </PanelShell>
  );
}

export function WorldSummaryPanel({
  snapshot,
  hideCognition = false,
}: {
  snapshot: ChannelSnapshot | undefined;
  hideCognition?: boolean;
}) {
  const payload = snapshot?.payload ?? {};
  const hint =
    payload.world_revision_hint && typeof payload.world_revision_hint === "object"
      ? JSON.stringify(payload.world_revision_hint)
      : "—";

  return (
    <PanelShell title="World summary">
      <div className="space-y-2">
        <Field label="entity_count" value={String(payload.entity_count ?? "—")} />
        <Field label="revision" value={String(payload.revision ?? "—")} />
        <Field label="sync_health" value={String(payload.sync_health ?? "—")} />
        <Field
          label="telemetry_health"
          value={String(payload.telemetry_health ?? "—")}
        />
        <Field label="world_revision_hint (explanatory)" value={hint} />
      </div>
      {!hideCognition && <TelemetryCognitionStrip snapshot={snapshot} />}
    </PanelShell>
  );
}

export function ClockMirrorPanel({
  snapshot,
  hideCognition = false,
}: {
  snapshot: ChannelSnapshot | undefined;
  hideCognition?: boolean;
}) {
  const payload = snapshot?.payload ?? {};
  return (
    <PanelShell title="Clock mirror">
      <div className="space-y-2">
        <Field label="paused" value={String(payload.paused ?? "—")} />
        <Field label="mode" value={String(payload.mode ?? "—")} />
      </div>
      {!hideCognition && <TelemetryCognitionStrip snapshot={snapshot} />}
    </PanelShell>
  );
}
