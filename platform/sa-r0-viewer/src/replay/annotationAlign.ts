import type { ReplaySaBundle } from "./bundleSchema";

export type AlignedAnnotation = {
  id: string;
  label: string;
  status: "matched" | "a_only" | "b_only";
  titleA?: string;
  titleB?: string;
};

function normLabel(ann: { title?: string; annotation_id?: string }): string {
  return String(ann.title ?? ann.annotation_id ?? "").trim().toLowerCase();
}

export function alignAnnotations(
  bundleA: ReplaySaBundle,
  bundleB: ReplaySaBundle,
): AlignedAnnotation[] {
  const annA = bundleA.narrative.annotations ?? [];
  const annB = bundleB.narrative.annotations ?? [];
  const byLabelB = new Map<string, (typeof annB)[number]>();
  for (const b of annB) {
    byLabelB.set(normLabel(b), b);
  }
  const usedB = new Set<string>();
  const rows: AlignedAnnotation[] = [];

  for (const a of annA) {
    const key = normLabel(a);
    const match = byLabelB.get(key);
    if (match) {
      usedB.add(normLabel(match));
      rows.push({
        id: String(a.annotation_id ?? key),
        label: String(a.title ?? a.annotation_id),
        status: "matched",
        titleA: String(a.title),
        titleB: String(match.title),
      });
    } else {
      rows.push({
        id: String(a.annotation_id ?? key),
        label: String(a.title ?? a.annotation_id),
        status: "a_only",
        titleA: String(a.title),
      });
    }
  }
  for (const b of annB) {
    const key = normLabel(b);
    if (usedB.has(key)) continue;
    rows.push({
      id: String(b.annotation_id ?? key),
      label: String(b.title ?? b.annotation_id),
      status: "b_only",
      titleB: String(b.title),
    });
  }
  return rows;
}

export function findAlignedEventId(
  bundleA: ReplaySaBundle,
  bundleB: ReplaySaBundle,
  eventIdA: string,
): string | null {
  const evA = bundleA.narrative.events.find((e) => e.event_id === eventIdA);
  if (!evA?.category) return null;
  const lineA = evA.line_index as number | null | undefined;
  const candidates = bundleB.narrative.events.filter((e) => e.category === evA.category);
  if (candidates.length === 0) return null;
  if (lineA == null) return String(candidates[0].event_id);
  let best = candidates[0];
  let bestDist = Infinity;
  for (const c of candidates) {
    const lineB = c.line_index as number | null | undefined;
    if (lineB == null) continue;
    const dist = Math.abs(lineB - lineA);
    if (dist < bestDist) {
      bestDist = dist;
      best = c;
    }
  }
  return String(best.event_id);
}
