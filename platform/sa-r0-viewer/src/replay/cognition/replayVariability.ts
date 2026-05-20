import type { ReplayMcSweep } from "../sweepSchema";

export function buildVariabilitySummaries(sweep: ReplayMcSweep): string[] {
  const out: string[] = [];
  const narrative = sweep.replay_narrative_summary;
  if (narrative?.bullets?.length) {
    out.push(...narrative.bullets);
  }
  const patterns = sweep.replay_aggregation?.dominant_patterns ?? [];
  for (const p of patterns) {
    if (!out.includes(p)) out.push(p);
  }

  const hist = sweep.replay_aggregation?.outcome_histogram ?? {};
  const fd = hist.first_detection_t ?? [];
  if (fd.length >= 2) {
    const min = Math.min(...fd);
    const max = Math.max(...fd);
    if (max - min >= 2) {
      out.push(
        `Detection timing varies by ${max - min} log-line indices across replay variants in this sweep.`,
      );
    }
  }
  const los = hist.los_degraded_count ?? [];
  if (los.length >= 2 && Math.max(...los) - Math.min(...los) >= 2) {
    out.push(
      "LOS degradation replay concentration differs across topology variants — see spatial overlays.",
    );
  }

  const clusters = sweep.spatial_aggregate.layers.replay_event_clusters;
  if (clusters?.labels?.length) {
    out.push(clusters.labels[0]!);
  }

  if (!out.length) {
    out.push(
      `Replay variants under ${sweep.baseline_topology_key} form an explanatory sweep family for spatial review.`,
    );
  }
  return out.slice(0, 5);
}
