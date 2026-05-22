import type { ReplaySaBundle } from "../bundleSchema";
import type { ReplayMcSweep } from "../sweepSchema";

export type StorytellingSection = {
  id: string;
  title: string;
  body: string;
};

export function buildBundleStorytelling(bundle: ReplaySaBundle): StorytellingSection[] {
  const sections: StorytellingSection[] = [];
  const topology = bundle.comparison_hints?.topology_key ?? "unknown";
  const tags = bundle.scenario.topology_tags ?? [];
  const duration = bundle.clock.duration;
  const span = duration.end - duration.start;
  const events = bundle.narrative.events ?? [];
  const windows = bundle.narrative.windows ?? [];
  const los = (bundle.los_segments ?? []).filter((s) =>
    ["terrain_blocked", "partially_occluded"].includes(s.status),
  ).length;

  sections.push({
    id: "what_changed",
    title: "What changed",
    body: `This replay variant uses topology \`${topology}\` (${tags.slice(0, 3).join(", ") || "no extra tags"}) over a ${span}-step log span — derived replay summary only.`,
  });

  sections.push({
    id: "topology_pacing",
    title: "How topology altered replay pacing",
    body: `Ingress archetype \`${bundle.scenario.ingress_archetype ?? "standard"}\` shapes replay pacing across ${events.length} narrative events.`,
  });

  if (los >= 1) {
    sections.push({
      id: "los_instability",
      title: "Where LOS instability occurred",
      body: `LOS instability appears in ${los} degraded segment(s) in this replay variant — review near terrain masking overlays; not sensor physics proof.`,
    });
  }

  if (windows.length >= 1) {
    sections.push({
      id: "ambiguity_concentration",
      title: "Where ambiguity concentrated",
      body: `Ambiguity concentrates in ${windows.length} replay window(s) during ingress overlap — explanatory spatial concentration only.`,
    });
  }

  const divEvents = events.filter((e) => e.category === "divergence");
  if (divEvents.length >= 1) {
    sections.push({
      id: "divergence_summary",
      title: "Why this replay diverged",
      body: `${divEvents.length} divergence event(s) appear in replay-local sequence — localized mismatch language only; not causal proof.`,
    });
  }

  return sections;
}

export function buildSweepStorytelling(sweep: ReplayMcSweep): StorytellingSection[] {
  const sections: StorytellingSection[] = [];
  const baseline = sweep.baseline_topology_key;
  const hist = sweep.replay_aggregation?.outcome_histogram ?? {};
  const fd = hist.first_detection_t ?? [];
  const members = sweep.members ?? [];

  if (fd.length >= 2 && Math.max(...fd) - Math.min(...fd) >= 2) {
    sections.push({
      id: "divergence_summary",
      title: "Why this replay diverged",
      body: `Detection timing differs by ${Math.max(...fd) - Math.min(...fd)} log-line indices across ${members.length} replay variants under \`${baseline}\`.`,
    });
  } else if (sweep.replay_aggregation?.dominant_patterns?.[0]) {
    sections.push({
      id: "divergence_summary",
      title: "Why this replay diverged",
      body: sweep.replay_aggregation.dominant_patterns[0]!,
    });
  }

  const ambCounts = sweep.spatial_aggregate.layers.ambiguity_density?.counts ?? [];
  if (ambCounts.length && Math.max(...ambCounts) >= 3) {
    sections.push({
      id: "ambiguity_concentration",
      title: "Where ambiguity concentrated",
      body: "Ambiguity density concentrates in shared spatial cells across sweep members — see ambiguity overlay in presentation mode.",
    });
  }

  const losCounts = sweep.spatial_aggregate.layers.los_degraded?.counts ?? [];
  if (losCounts.length && Math.max(...losCounts) >= 2) {
    sections.push({
      id: "los_instability",
      title: "Where LOS instability occurred",
      body: "LOS degradation replay concentration differs across topology variants in this sweep.",
    });
  }

  const nonBaseline = members.filter((m) => m.pack_id !== baseline).length;
  sections.push({
    id: "what_changed",
    title: "What changed",
    body: `${nonBaseline} non-baseline pack(s) diverge from \`${baseline}\` in this sweep family.`,
  });

  return sections;
}

export type CognitionIndicators = {
  annotationDensity: string;
  replayPacing: string;
  overlayClutter: string;
  narrativeComplexity: string;
};

export function computeCognitionIndicators(bundle: ReplaySaBundle | null, sweep: ReplayMcSweep | null): CognitionIndicators {
  const annotations = bundle?.narrative.annotations?.length ?? 0;
  const span = bundle ? Math.max(1, bundle.clock.duration.end - bundle.clock.duration.start) : 1;
  const events = bundle?.narrative.events?.length ?? 0;
  const density = (annotations / span).toFixed(2);

  const pacing =
    events >= 10
      ? "Dense event cadence — consider chapter navigation."
      : events >= 5
        ? "Moderate event cadence."
        : "Sparse event cadence.";

  let layerCount = 0;
  if (bundle?.spatial_analytics) layerCount += 2;
  if (sweep?.spatial_aggregate) layerCount += 3;
  const clutter =
    layerCount >= 4
      ? "High overlay density — spatial declutter recommended."
      : layerCount >= 2
        ? "Moderate overlay density."
        : "Low overlay density.";

  const cohorts = sweep?.replay_cohorts?.length ?? 0;
  const tags = new Set(sweep?.members?.flatMap((m) => m.replay_pattern_tags ?? []) ?? []);
  const complexity =
    cohorts >= 3 || tags.size >= 3
      ? "High narrative complexity — use guided walkthrough."
      : cohorts >= 1
        ? "Moderate narrative complexity."
        : "Low narrative complexity.";

  return {
    annotationDensity: `${density} annotations per log-line step`,
    replayPacing: pacing,
    overlayClutter: clutter,
    narrativeComplexity: complexity,
  };
}

export function importanceWeightEntries(weights: Record<string, number> | undefined): { key: string; weight: number }[] {
  if (!weights) return [];
  return Object.entries(weights)
    .sort((a, b) => b[1] - a[1])
    .map(([key, weight]) => ({ key, weight }));
}
