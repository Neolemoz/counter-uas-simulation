import {
  experimentAnalyticsReportSchema,
  experimentFidelityMetricsReportSchema,
  experimentManifestSchema,
  experimentMetricsReportSchema,
  experimentSpecSchema,
  type ExperimentAnalyticsReport,
  type ExperimentFidelityMetricsReport,
  type ExperimentManifest,
  type ExperimentMetricsReport,
  type ExperimentSpec,
} from "./experimentSchema";
import {
  experimentAnnexBundleSchema,
  rtTacticalCaptureAnnexSchema,
  type ExperimentAnnexBundle,
  type TacticalCaptureAnnex,
} from "./tacticalAnnexSchema";

export type ParseResult<T> =
  | { ok: true; data: T }
  | { ok: false; error: string };

function zodMessage(err: unknown): string {
  if (err && typeof err === "object" && "message" in err) {
    return String((err as { message: string }).message);
  }
  return "validation failed";
}

export function formatImportError(error: string): string {
  const trimmed = error.trim();
  if (trimmed.length > 120) {
    return `${trimmed.slice(0, 117)}...`;
  }
  return trimmed;
}

export function safeParseManifest(text: string): ParseResult<ExperimentManifest> {
  try {
    const raw = JSON.parse(text);
    return { ok: true, data: experimentManifestSchema.parse(raw) };
  } catch (err) {
    return { ok: false, error: zodMessage(err) };
  }
}

export function safeParseAnalyticsReport(
  text: string,
): ParseResult<ExperimentAnalyticsReport> {
  try {
    const raw = JSON.parse(text);
    return { ok: true, data: experimentAnalyticsReportSchema.parse(raw) };
  } catch (err) {
    return { ok: false, error: zodMessage(err) };
  }
}

export function safeParseAnnex(text: string): ParseResult<TacticalCaptureAnnex> {
  try {
    const raw = JSON.parse(text);
    return { ok: true, data: rtTacticalCaptureAnnexSchema.parse(raw) };
  } catch (err) {
    return { ok: false, error: zodMessage(err) };
  }
}

export function safeParseAnnexBundle(text: string): ParseResult<ExperimentAnnexBundle> {
  try {
    const raw = JSON.parse(text);
    return { ok: true, data: experimentAnnexBundleSchema.parse(raw) };
  } catch (err) {
    return { ok: false, error: zodMessage(err) };
  }
}

export function safeParseExperimentSpec(text: string): ParseResult<ExperimentSpec> {
  try {
    const raw = JSON.parse(text);
    return { ok: true, data: experimentSpecSchema.parse(raw) };
  } catch (err) {
    return { ok: false, error: zodMessage(err) };
  }
}

export function safeParseMetricsReport(text: string): ParseResult<ExperimentMetricsReport> {
  try {
    const raw = JSON.parse(text);
    return { ok: true, data: experimentMetricsReportSchema.parse(raw) };
  } catch (err) {
    return { ok: false, error: zodMessage(err) };
  }
}

export function safeParseFidelityMetricsReport(
  text: string,
): ParseResult<ExperimentFidelityMetricsReport> {
  try {
    const raw = JSON.parse(text);
    return { ok: true, data: experimentFidelityMetricsReportSchema.parse(raw) };
  } catch (err) {
    return { ok: false, error: zodMessage(err) };
  }
}
