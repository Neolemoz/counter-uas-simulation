import type { ReplaySaBundle } from "./bundleSchema";

export type TopologyDiffHighlight = {
  entityIds: string[];
  overlayIds: string[];
  zoneIds: string[];
};

export type TopologyDiffResult = {
  bullets: string[];
  highlight: TopologyDiffHighlight;
  tagIntersection: string[];
  tagOnlyA: string[];
  tagOnlyB: string[];
};

type Entity = ReplaySaBundle["entities_static"][number];
type Zone = ReplaySaBundle["zones"][number];
type Overlay = ReplaySaBundle["overlays"][number];

function posKey(pos: number[] | undefined): string {
  const p = pos ?? [0, 0, 0];
  return p.map((v) => v.toFixed(1)).join(",");
}

function entityMap(entities: Entity[]): Map<string, Entity> {
  const m = new Map<string, Entity>();
  for (const e of entities) {
    if (e.entity_id) m.set(e.entity_id, e);
  }
  return m;
}

function diffEntities(a: Entity[], b: Entity[], labelA: string, labelB: string): string[] {
  const ma = entityMap(a);
  const mb = entityMap(b);
  const bullets: string[] = [];
  const added = [...mb.keys()].filter((id) => !ma.has(id)).sort();
  const removed = [...ma.keys()].filter((id) => !mb.has(id)).sort();
  for (const id of added) {
    const ent = mb.get(id)!;
    bullets.push(
      `${labelB} adds static site ${id} (${ent.kind}) at ENU [${posKey(ent.position_enu_m)}].`,
    );
  }
  for (const id of removed) {
    bullets.push(`${labelB} omits static site ${id} present in ${labelA}.`);
  }
  for (const id of [...ma.keys()].filter((k) => mb.has(k)).sort()) {
    const pa = ma.get(id)!.position_enu_m ?? [0, 0, 0];
    const pb = mb.get(id)!.position_enu_m ?? [0, 0, 0];
    const dx = (pb[0] ?? 0) - (pa[0] ?? 0);
    const dy = (pb[1] ?? 0) - (pa[1] ?? 0);
    const dz = (pb[2] ?? 0) - (pa[2] ?? 0);
    if (Math.abs(dx) > 0.5 || Math.abs(dy) > 0.5 || Math.abs(dz) > 0.5) {
      bullets.push(
        `${id} shifted ${dx.toFixed(0)} m E, ${dy.toFixed(0)} m N, ${dz.toFixed(0)} m U vs ${labelA}.`,
      );
    }
  }
  return bullets;
}

function diffZones(a: Zone[], b: Zone[], labelA: string, labelB: string): string[] {
  const za = new Map(a.map((z) => [z.zone_id, z]));
  const zb = new Map(b.map((z) => [z.zone_id, z]));
  const bullets: string[] = [];
  for (const id of [...zb.keys()].filter((k) => !za.has(k)).sort()) {
    bullets.push(`${labelB} introduces zone ${id} (${zb.get(id)!.kind}).`);
  }
  for (const id of [...za.keys()].filter((k) => zb.has(k)).sort()) {
    const ga = za.get(id)!.geometry;
    const gb = zb.get(id)!.geometry;
    if (ga?.type === "circle" && gb?.type === "circle") {
      const ra = ga.radius_m ?? 0;
      const rb = gb.radius_m ?? 0;
      if (Math.abs(ra - rb) > 1) {
        bullets.push(`Zone ${id} radius differs: ${labelA} ${ra} m vs ${labelB} ${rb} m.`);
      }
    }
  }
  return bullets;
}

function diffOverlays(a: Overlay[], b: Overlay[], labelA: string, labelB: string): string[] {
  const oa = new Map(a.map((o) => [o.overlay_id, o]));
  const ob = new Map(b.map((o) => [o.overlay_id, o]));
  const bullets: string[] = [];
  for (const id of [...ob.keys()].filter((k) => !oa.has(k)).sort()) {
    const ov = ob.get(id)!;
    const range = ov.active_t_range ? ` active t=${ov.active_t_range.join("–")}` : "";
    bullets.push(`${labelB} adds ${ov.kind} overlay ${id}${range}.`);
  }
  for (const id of [...oa.keys()].filter((k) => !ob.has(k)).sort()) {
    bullets.push(`${labelB} removes overlay ${id} present in ${labelA}.`);
  }
  return bullets;
}

function diffLos(a: ReplaySaBundle, b: ReplaySaBundle): string[] {
  const countStatus = (bundle: ReplaySaBundle, status: string) =>
    (bundle.los_segments ?? []).filter((s) => s.status === status).length;
  const visA = countStatus(a, "visible");
  const visB = countStatus(b, "visible");
  const blkA = countStatus(a, "terrain_blocked");
  const blkB = countStatus(b, "terrain_blocked");
  const bullets: string[] = [];
  if (blkB !== blkA || visB !== visA) {
    bullets.push(
      `LOS segment counts (explanatory): visible ${visA}→${visB}, terrain_blocked ${blkA}→${blkB}.`,
    );
  }
  return bullets;
}

export function computeTopologyDiff(
  bundleA: ReplaySaBundle,
  bundleB: ReplaySaBundle,
  labelA = "Scenario A",
  labelB = "Scenario B",
): TopologyDiffResult {
  const bullets = [
    ...diffEntities(bundleA.entities_static, bundleB.entities_static, labelA, labelB),
    ...diffZones(bundleA.zones, bundleB.zones, labelA, labelB),
    ...diffOverlays(bundleA.overlays, bundleB.overlays, labelA, labelB),
    ...diffLos(bundleA, bundleB),
  ];
  if (bullets.length === 0) {
    bullets.push("No topology diff detected in static sites, zones, or overlays.");
  }

  const ma = entityMap(bundleA.entities_static);
  const mb = entityMap(bundleB.entities_static);
  const entityIds: string[] = [];
  for (const id of new Set([...ma.keys(), ...mb.keys()])) {
    if (!ma.has(id) || !mb.has(id)) {
      entityIds.push(id);
      continue;
    }
    if (posKey(ma.get(id)!.position_enu_m) !== posKey(mb.get(id)!.position_enu_m)) {
      entityIds.push(id);
    }
  }

  const oa = new Set(bundleA.overlays.map((o) => o.overlay_id));
  const ob = new Set(bundleB.overlays.map((o) => o.overlay_id));
  const overlayIds = [...new Set([...oa, ...ob].filter((id) => !oa.has(id) || !ob.has(id)))];

  const za = new Set(bundleA.zones.map((z) => z.zone_id));
  const zb = new Set(bundleB.zones.map((z) => z.zone_id));
  const zoneIds = [...new Set([...za, ...zb].filter((id) => !za.has(id) || !zb.has(id)))];

  const tagsA = new Set(bundleA.scenario.topology_tags ?? []);
  const tagsB = new Set(bundleB.scenario.topology_tags ?? []);

  return {
    bullets,
    highlight: { entityIds, overlayIds, zoneIds },
    tagIntersection: [...tagsA].filter((t) => tagsB.has(t)).sort(),
    tagOnlyA: [...tagsA].filter((t) => !tagsB.has(t)).sort(),
    tagOnlyB: [...tagsB].filter((t) => !tagsA.has(t)).sort(),
  };
}
