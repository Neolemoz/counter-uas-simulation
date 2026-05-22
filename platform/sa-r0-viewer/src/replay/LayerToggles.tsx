import type { CSSProperties } from "react";
import { useClockStore, type LayerVisibility } from "./clockStore";
import { useCompareStore } from "./compareStore";
import { useSweepStore, type SpatialDeclutterMode } from "./useSweepStore";
import { overlayFillColor } from "@/cesium/overlayStyles";

const LABELS: Record<Exclude<keyof LayerVisibility, "spatial">, string> = {
  sites: "Sensor sites & bases",
  tracks: "Tracks (dashed)",
  zones: "Threat/risk zones",
  overlays: "Terrain masking overlays",
  losLinks: "LOS links (explanatory)",
  narrativeMarkers: "Narrative markers",
};

const SPATIAL_LABELS: Record<keyof LayerVisibility["spatial"], string> = {
  spatialAmbiguity: "Ambiguity concentration",
  spatialLos: "LOS degradation concentration",
  spatialSensitivity: "Topology sensitivity",
};

const OVERLAY_KIND_LABELS: Record<string, string> = {
  ridge_mask: "Ridge mask",
  los_blocked: "LOS blocked",
  degraded_visibility: "Degraded visibility",
  ingress_corridor: "Ingress corridor",
};

function swatchStyle(kind: string): CSSProperties {
  const c = overlayFillColor(kind);
  return {
    backgroundColor: c.toCssColorString(),
    width: 12,
    height: 12,
    borderRadius: 2,
    border: "1px solid rgba(148,163,184,0.5)",
  };
}

export function LayerToggles() {
  const bundle = useClockStore((s) => s.bundle);
  const layers = useClockStore((s) => s.layers);
  const losScope = useClockStore((s) => s.losScope);
  const toggleLayer = useClockStore((s) => s.toggleLayer);
  const toggleSpatialLayer = useClockStore((s) => s.toggleSpatialLayer);
  const setLosScope = useClockStore((s) => s.setLosScope);
  const compareMode = useCompareStore((s) => s.mode === "compare");
  const focusSlot = useCompareStore((s) => s.focusSlot);
  const sweepMode = useSweepStore((s) => s.mode === "sweep");
  const spatialDeclutter = useSweepStore((s) => s.spatialDeclutter);
  const setSpatialDeclutter = useSweepStore((s) => s.setSpatialDeclutter);
  const toggleSlotLayer = useCompareStore((s) => s.toggleSlotLayer);
  const setSlotLosScope = useCompareStore((s) => s.setSlotLosScope);

  const overlayKinds =
    bundle?.scenario.overlay_descriptors?.map((d) => d.kind) ??
    [...new Set((bundle?.overlays ?? []).map((o) => o.kind))];

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-2 font-semibold text-slate-200">Layers</h2>
      <ul className="space-y-1">
        {(Object.keys(LABELS) as (Exclude<keyof LayerVisibility, "spatial">)[]).map((key) => (
          <li key={key}>
            <label className="flex cursor-pointer items-center gap-2 text-slate-300">
              <input
                type="checkbox"
                checked={layers[key]}
                onChange={() => {
                  if (compareMode) toggleSlotLayer(focusSlot, key);
                  toggleLayer(key);
                }}
              />
              {LABELS[key]}
            </label>
          </li>
        ))}
      </ul>
      <div className="mt-2 border-t border-slate-700 pt-2">
        <p className="mb-1 text-xs uppercase text-slate-500">Spatial analytics (replay)</p>
        <ul className="space-y-1">
          {(Object.keys(SPATIAL_LABELS) as (keyof LayerVisibility["spatial"])[]).map((key) => (
            <li key={key}>
              <label className="flex cursor-pointer items-center gap-2 text-slate-300">
                <input
                  type="checkbox"
                  checked={layers.spatial[key]}
                  onChange={() => toggleSpatialLayer(key)}
                />
                {SPATIAL_LABELS[key]}
              </label>
            </li>
          ))}
        </ul>
        {sweepMode && (
          <div className="mt-2">
            <p className="mb-1 text-xs text-slate-500">Overlay declutter (sweep)</p>
            <select
              className="w-full rounded bg-slate-800 px-2 py-1 text-xs text-slate-300"
              value={spatialDeclutter}
              onChange={(e) => setSpatialDeclutter(e.target.value as SpatialDeclutterMode)}
            >
              <option value="top_k">Top cells (12)</option>
              <option value="threshold">P75 threshold</option>
              <option value="off">Show all</option>
            </select>
          </div>
        )}
      </div>
      {layers.losLinks && (
        <div className="mt-2 border-t border-slate-700 pt-2">
          <p className="mb-1 text-xs uppercase text-slate-500">LOS scope</p>
          <label className="mr-3 inline-flex items-center gap-1 text-xs text-slate-400">
            <input
              type="radio"
              name="losScope"
              checked={losScope === "all"}
              onChange={() => {
                if (compareMode) setSlotLosScope(focusSlot, "all");
                setLosScope("all");
              }}
            />
            All tracks
          </label>
          <label className="inline-flex items-center gap-1 text-xs text-slate-400">
            <input
              type="radio"
              name="losScope"
              checked={losScope === "selected_track"}
              onChange={() => {
                if (compareMode) setSlotLosScope(focusSlot, "selected_track");
                setLosScope("selected_track");
              }}
            />
            Primary threat
          </label>
        </div>
      )}
      {overlayKinds.length > 0 && (
        <div className="mt-2 border-t border-slate-700 pt-2">
          <p className="mb-1 text-xs uppercase text-slate-500">Overlay legend</p>
          <ul className="space-y-1">
            {overlayKinds.map((kind) => (
              <li key={kind} className="flex items-center gap-2 text-xs text-slate-400">
                <span style={swatchStyle(kind)} aria-hidden />
                {OVERLAY_KIND_LABELS[kind] ?? kind}
              </li>
            ))}
          </ul>
        </div>
      )}
    </section>
  );
}
