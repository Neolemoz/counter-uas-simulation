/** Canonical governance banner strings (PLAT-RT-T1). */

export const BANNER_PRIMARY =
  "RT SANDBOX — experimental simulation; not operational state";

export const BANNER_TRANSIENT =
  "TRANSIENT RUNTIME ONLY — not replay authority";

export const BANNER_NOT_SA = "NOT SA REPLAY AUTHORITY";

export const BANNER_MANUAL_HANDOFF_ONLY = "MANUAL HANDOFF ONLY";

export const BANNER_WORLD_EDITING = "WORLD EDITING ACTIVE";

export const BANNER_CESIUM = "CESIUM RUNTIME VIEW";

export const BANNER_INTERACTIVE_EDITING = "INTERACTIVE EDITING";

export const BANNER_MULTI_SESSION =
  "MULTI-SESSION — local prototype; not operational coordination";

/** Additive panel banner (PLAT-RT-V2) — does not replace frozen T1 strings. */
export const BANNER_TERRAIN =
  "TERRAIN VISUALIZATION — fictional sandbox geometry; not sensor or terrain truth";

/** Additive panel banner (PLAT-RT-F4) — contours / vegetation layers. */
export const BANNER_REALISM_F4 =
  "REALISM LAYERS — explanatory contours and vegetation cues; not survey or sensor truth";

/** Additive panel banner (PLAT-RT-X1). */
export const BANNER_EXPERIMENT =
  "RT EXPERIMENT — explanatory compare only; not operational authority";

/** Additive panel banner (PLAT-RT-X2 P0). */
export const BANNER_EXPERIMENT_V2 =
  "RT EXPERIMENT v2 — cohort review is explanatory; maintainer CLIs remain authority";

/** Additive panel banner (PLAT-RT-F1). */
export const BANNER_ANALYTICS =
  "RT ANALYTICS — derived summaries only; not operational authority";

/** Additive panel banner (PLAT-RT-F3). */
export const BANNER_ANNEX_REVIEW =
  "RT ANNEX REVIEW — replay-boundary timelines only; not operational authority";

/** Additive panel banner (PLAT-RT-F5 P1). */
export const BANNER_EXPERIMENT_F5 =
  "RT EXPERIMENT — derived summaries only; not operational authority";

/** Additive panel banner (PLAT-RT-F5b P1). */
export const BANNER_FIDELITY_TRUTH =
  "RT FIDELITY TRUTH — sim-scoped attestation only; not SA replay or operational sensor authority";

/** Additive panel banner (PLAT-RT-F6 P1). */
export const BANNER_SA_WORKFLOW_ADVISORY =
  "SA WORKFLOW ADVISORY — explanatory only; maintainer CLIs are authority";

/** Additive panel banner (PLAT-RT-V3 P1) — visibility overlays. */
export const BANNER_VISIBILITY_V3 =
  "VISIBILITY OVERLAYS — heuristic wedge/horizon/LOS; not sensor coverage or operational picture";

export const BASE_BANNERS = [
  BANNER_PRIMARY,
  BANNER_TRANSIENT,
  BANNER_NOT_SA,
  BANNER_MANUAL_HANDOFF_ONLY,
] as const;

export const CONNECTED_BANNERS = [
  ...BASE_BANNERS,
  BANNER_WORLD_EDITING,
  BANNER_CESIUM,
  BANNER_INTERACTIVE_EDITING,
] as const;

/** @deprecated Use CONNECTED_BANNERS — kept for T2 contract references */
export const ALL_BANNERS = CONNECTED_BANNERS;

export function bannersForSession(
  connected: boolean,
  multiSession = false,
): readonly string[] {
  if (!connected) return BASE_BANNERS;
  const banners: string[] = [...CONNECTED_BANNERS];
  if (multiSession) banners.push(BANNER_MULTI_SESSION);
  return banners;
}

/** Forbidden lexicon for UI copy checks (rt_runtime_governance_v1 §7). */
export const FORBIDDEN_LEXICON = [
  "engage",
  "intercept",
  "strike",
  "tactical readiness",
  "mission success",
  "threat neutralized",
] as const;

/** Additional forbidden terms for F6 advisory surfaces (rt_sa_workflow_advisory_ui_v1 §3.3). */
export const ADVISORY_FORBIDDEN_LEXICON = [
  ...FORBIDDEN_LEXICON,
  "readiness_score",
  "auto_import",
  "automatic import",
  "operational_ready",
  "operational readiness",
  "winner",
  "success_rate",
] as const;
