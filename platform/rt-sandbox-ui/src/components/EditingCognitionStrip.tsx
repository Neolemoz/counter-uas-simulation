import { TelemetryCognitionStrip } from "./TelemetryCognitionStrip";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import {
  describeCommandIntent,
  describeMirrorLag,
  formatCommandResult,
} from "@/editing/cognition";
import type { EditCommandType } from "@/editing/editHistory";

export function EditingCognitionStrip({
  lastCommand,
  pendingReconcile,
  mirrorSnapshot,
}: {
  lastCommand?: {
    type: EditCommandType;
    ok: boolean;
    errorCode?: string;
    message?: string;
  };
  pendingReconcile: boolean;
  mirrorSnapshot: ChannelSnapshot | undefined;
}) {
  return (
    <div className="mt-3 space-y-3 rounded border border-emerald-800/50 bg-emerald-950/20 p-3">
      <p className="text-xs font-semibold uppercase text-emerald-300/90">
        Editing cognition
      </p>
      {lastCommand && (
        <div className="text-xs text-slate-300">
          <p>{describeCommandIntent(lastCommand.type)}</p>
          <p className="text-slate-400">
            {formatCommandResult(
              lastCommand.ok,
              lastCommand.errorCode,
              lastCommand.message,
            )}
          </p>
        </div>
      )}
      <p className="text-xs italic text-slate-500">{describeMirrorLag(pendingReconcile)}</p>
      <div>
        <p className="mb-1 text-xs text-slate-500">Adapter feedback / telemetry mirror:</p>
        <TelemetryCognitionStrip snapshot={mirrorSnapshot} />
      </div>
    </div>
  );
}
