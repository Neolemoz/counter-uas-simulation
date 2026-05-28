import type { ReactNode } from "react";
import { BANNER_EXPERIMENT_V3 } from "@/governance/banners";
import type { ExperimentCohortIndex } from "./cohortSchema";
import { ExperimentManifestRoster } from "./ExperimentManifestRoster";
import { ExperimentProgramContextStrip } from "./ExperimentProgramContextStrip";
import { ExperimentSecondaryManifestPicker } from "./ExperimentSecondaryManifestPicker";
import type { ExperimentManifest } from "./experimentSchema";
import type { WorkbenchV2State } from "./workbenchV2State";

export function ExperimentWorkbenchV3Shell({
  v2State,
  cohort,
  loadedManifest,
  onV2StateChange,
  children,
}: {
  v2State: WorkbenchV2State;
  cohort: ExperimentCohortIndex | null;
  loadedManifest: ExperimentManifest;
  onV2StateChange: (state: WorkbenchV2State) => void;
  children: ReactNode;
}) {
  return (
    <div className="space-y-3" data-testid="experiment-workbench-v3">
      <p className="text-[10px] text-amber-200/90">{BANNER_EXPERIMENT_V3}</p>
      <ExperimentProgramContextStrip
        v2State={v2State}
        cohort={cohort}
        onV2StateChange={onV2StateChange}
      />
      <ExperimentManifestRoster
        v2State={v2State}
        cohort={cohort}
        loadedManifest={loadedManifest}
        onV2StateChange={onV2StateChange}
      />
      <ExperimentSecondaryManifestPicker
        v2State={v2State}
        cohort={cohort}
        onV2StateChange={onV2StateChange}
      />
      {children}
    </div>
  );
}
