export const TRACK_LIVE_ENTITY_POSE_MIRROR_AUTHORITY = "entity_pose_mirror_explanatory";
export const TRACK_LIVE_INTELLIGENCE_ADVISORY_AUTHORITY =
  "intelligence_advisory_explanatory";
export const TRACK_LIVE_ADAPTER_AUTHORITY = "track_live_adapter_explanatory";

export const TRACK_LIVE_WORKBENCH_COPY = "Live entity mirror workbench";
export const TRACK_LIVE_NOT_TRACKER_OUTPUT_COPY = "Not tracker output";
export const TRACK_LIVE_NO_TRACK_TELEMETRY_COPY = "No live track telemetry";
export const TRACK_LIVE_READ_ONLY_COPY = "Read-only explanation surface";
export const TRACK_LIVE_NO_ASSIGNMENT_AUTHORITY_COPY = "No assignment authority";
export const TRACK_LIVE_NO_ENGAGEMENT_AUTHORITY_COPY = "No engagement authority";
export const TRACK_LIVE_NO_AUTONOMY_AUTHORITY_COPY = "No autonomy authority";

export const TRACK_LIVE_GOVERNANCE_LINES = [
  TRACK_LIVE_WORKBENCH_COPY,
  TRACK_LIVE_NOT_TRACKER_OUTPUT_COPY,
  TRACK_LIVE_NO_TRACK_TELEMETRY_COPY,
  TRACK_LIVE_READ_ONLY_COPY,
  TRACK_LIVE_NO_ASSIGNMENT_AUTHORITY_COPY,
  TRACK_LIVE_NO_ENGAGEMENT_AUTHORITY_COPY,
  TRACK_LIVE_NO_AUTONOMY_AUTHORITY_COPY,
] as const;

export const TRACK_LIVE_DEGRADED_CONFIDENCE_BASIS =
  "No tracker confidence telemetry exists; no sensor fusion telemetry exists; no tracker lifecycle telemetry exists. Live entity pose mirror is explanatory only.";

export const TRACK_LIVE_SENSOR_UNAVAILABLE_NOTE =
  "No live sensor contribution telemetry exists for this workbench.";

export const TRACK_LIVE_LIFECYCLE_UNAVAILABLE_NOTE =
  "No live tracker lifecycle telemetry exists for this workbench.";
