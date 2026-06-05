export const TRACEABILITY_ENTITY_POSE_MIRROR_AUTHORITY = "entity_pose_mirror_explanatory";
export const TRACEABILITY_INTELLIGENCE_ADVISORY_AUTHORITY =
  "intelligence_advisory_explanatory";
export const TRACEABILITY_LIVE_ADAPTER_AUTHORITY =
  "traceability_live_adapter_explanatory";

export const TRACEABILITY_NOT_TRACKER_LINEAGE_COPY =
  "Live traceability is entity/advisory correlation only; it is not tracker lineage.";
export const TRACEABILITY_RECOMMENDATION_ORIGIN_COPY =
  "Recommendation origin visibility only.";
export const TRACEABILITY_READ_ONLY_COPY = "Read-only explanation surface.";
export const TRACEABILITY_NO_ASSIGNMENT_AUTHORITY_COPY = "No assignment authority.";
export const TRACEABILITY_NO_ENGAGEMENT_AUTHORITY_COPY = "No engagement authority.";
export const TRACEABILITY_NO_AUTONOMY_AUTHORITY_COPY = "No autonomy authority.";

export const TRACEABILITY_LIVE_GOVERNANCE_LINES = [
  TRACEABILITY_NOT_TRACKER_LINEAGE_COPY,
  TRACEABILITY_RECOMMENDATION_ORIGIN_COPY,
  TRACEABILITY_READ_ONLY_COPY,
  TRACEABILITY_NO_ASSIGNMENT_AUTHORITY_COPY,
  TRACEABILITY_NO_ENGAGEMENT_AUTHORITY_COPY,
  TRACEABILITY_NO_AUTONOMY_AUTHORITY_COPY,
] as const;

export const TRACEABILITY_DEGRADED_CONFIDENCE_BASIS =
  "No tracker confidence available; no sensor fusion evidence available; no tracker lifecycle available. Live entity pose mirror is explanatory only.";
