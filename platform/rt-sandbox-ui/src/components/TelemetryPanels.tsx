import { Activity } from "lucide-react";
import { pickAdapterFields } from "@/adapter/adapterStatus";
import { PanelShell } from "./GovernanceChrome";
import { TelemetryCognitionStrip } from "./TelemetryCognitionStrip";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";

function Field({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex justify-between gap-3 text-xs">
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
    <PanelShell title="Session lifecycle" icon={Activity} variant="tertiary">
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
  const adapter = pickAdapterFields(payload as Record<string, unknown>);

  return (
    <PanelShell title="Session health" icon={Activity} variant="tertiary">
      <div className="space-y-2">
        <Field label="state" value={String(payload.state ?? "—")} />
        <Field
          label="stub_alive"
          value={
            adapter.stubAlive === null ? "—" : adapter.stubAlive ? "yes" : "no"
          }
        />
        <Field
          label="adapter_alive"
          value={
            adapter.adapterAlive === null
              ? "—"
              : adapter.adapterAlive
                ? "yes"
                : "no"
          }
        />
        <Field label="adapter_mode" value={adapter.adapterMode ?? "—"} />
        <Field label="adapter_pid" value={adapter.adapterPid ?? "—"} />
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
    <PanelShell title="World summary" icon={Activity} variant="tertiary">
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
    <PanelShell title="Clock mirror" icon={Activity} variant="tertiary">
      <div className="space-y-2">
        <Field label="paused" value={String(payload.paused ?? "—")} />
        <Field label="mode" value={String(payload.mode ?? "—")} />
      </div>
      {!hideCognition && <TelemetryCognitionStrip snapshot={snapshot} />}
    </PanelShell>
  );
}
