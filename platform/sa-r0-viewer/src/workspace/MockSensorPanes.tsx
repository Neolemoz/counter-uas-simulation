import { RadarMockPane } from "@/views/RadarMockPane";
import { EoIrMockPane } from "@/views/EoIrMockPane";
import { InterceptorCameraMockPane } from "@/views/InterceptorCameraMockPane";
import { TelemetryPanel } from "@/views/TelemetryPanel";
import { ThreatAssessmentPanel } from "@/views/ThreatAssessmentPanel";
import { useActiveReplaySlot } from "./useActiveReplaySlot";

/** Mock panes driven by active replay slot (main or compare focus). */
export function MockSensorPanes() {
  const slot = useActiveReplaySlot();
  if (!slot.bundle) {
    return <p className="text-xs text-slate-500">Load a replay bundle to preview illustrative sensors.</p>;
  }
  return (
    <div className="grid gap-3">
      <RadarMockPane bundle={slot.bundle} currentT={slot.currentT} />
      <EoIrMockPane bundle={slot.bundle} currentT={slot.currentT} />
      <InterceptorCameraMockPane bundle={slot.bundle} currentT={slot.currentT} />
      <TelemetryPanel bundle={slot.bundle} />
      <ThreatAssessmentPanel bundle={slot.bundle} />
    </div>
  );
}
