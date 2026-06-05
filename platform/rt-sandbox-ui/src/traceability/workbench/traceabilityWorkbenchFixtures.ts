import { TRACEABILITY_FIXTURE_INPUTS } from "./traceabilityFixtureInputs";
import { assembleTraceabilityWorkbenchModel } from "./traceabilitySelectors";
import type { TraceabilityWorkbenchModel } from "./traceabilityWorkbenchTypes";

export const FULLY_LINKED_LINEAGE_FIXTURE: TraceabilityWorkbenchModel =
  assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.fullyLinked);

export const PARTIAL_LINEAGE_FIXTURE: TraceabilityWorkbenchModel =
  assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.partial);

export const MISSING_ADVISORY_FIXTURE: TraceabilityWorkbenchModel =
  assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.missingAdvisory);

export const STALE_ADVISORY_FIXTURE: TraceabilityWorkbenchModel =
  assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.staleAdvisory);

export const MISMATCH_LINEAGE_FIXTURE: TraceabilityWorkbenchModel =
  assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.mismatch);

export const TRACEABILITY_WORKBENCH_FIXTURES: readonly TraceabilityWorkbenchModel[] = [
  FULLY_LINKED_LINEAGE_FIXTURE,
  PARTIAL_LINEAGE_FIXTURE,
  MISSING_ADVISORY_FIXTURE,
  STALE_ADVISORY_FIXTURE,
  MISMATCH_LINEAGE_FIXTURE,
];
