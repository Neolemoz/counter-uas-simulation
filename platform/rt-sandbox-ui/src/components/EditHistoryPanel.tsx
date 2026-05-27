import type { EditHistoryEntry } from "@/editing/editHistory";
import { describeCommandIntent } from "@/editing/cognition";
import { PanelShell } from "./GovernanceChrome";

export function EditHistoryPanel({ history }: { history: EditHistoryEntry[] }) {
  return (
    <PanelShell title="Edit history (session-local)">
      <p className="mb-2 text-xs text-slate-500">
        Not persisted — cleared on disconnect. Command-authoritative registry log.
      </p>
      {history.length === 0 ? (
        <p className="text-sm text-slate-500">No edits yet.</p>
      ) : (
        <ul className="max-h-48 space-y-2 overflow-y-auto text-xs font-mono">
          {history.map((entry) => (
            <li
              key={entry.id}
              className={`rounded border px-2 py-1 ${
                entry.ok
                  ? "border-slate-700 text-slate-300"
                  : "border-red-800 text-red-300"
              }`}
            >
              <div>
                {entry.timestampUtc} · {entry.commandType}{" "}
                {entry.ok ? "OK" : entry.errorCode}
              </div>
              <div className="text-slate-500">{describeCommandIntent(entry.commandType)}</div>
              {entry.entityId && (
                <div>
                  entity {entry.entityId.slice(0, 8)}…
                  {entry.pose &&
                    ` @ (${entry.pose.x.toFixed(0)}, ${entry.pose.y.toFixed(0)}, ${entry.pose.z})`}
                </div>
              )}
            </li>
          ))}
        </ul>
      )}
    </PanelShell>
  );
}
