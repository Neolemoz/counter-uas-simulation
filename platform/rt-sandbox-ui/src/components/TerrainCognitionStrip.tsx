import type { MirrorEntity } from "@/cesium/entityMarkers";
import {
  entityTerrainRelation,
  losCueSummary,
  nearestOcclusionTarget,
  sensorDomeContext,
  terrainContextLine,
  visibilityHint,
} from "@/cesium/terrainCognition";
import type { FidelityContext } from "@/fidelity/fidelityCognition";
import { isFidelityCouplingOn, simAglForEntity } from "@/fidelity/fidelityCognition";
import { BANNER_FIDELITY_TRUTH } from "@/governance/banners";
import { TERRAIN_CAUTION } from "@/cesium/rtFictionalTerrain";

export function TerrainCognitionStrip({
  selectedEntity,
  entities,
  layersEnabled,
  fidelityContext,
}: {
  selectedEntity: MirrorEntity | null;
  entities: MirrorEntity[];
  layersEnabled: boolean;
  fidelityContext?: FidelityContext;
}) {
  if (!layersEnabled) {
    return (
      <p className="mt-2 text-[10px] text-slate-600">
        Terrain cognition off — enable terrain layers to see ridge / occlusion context.
      </p>
    );
  }

  return (
    <div className="mt-2 space-y-2 rounded border border-slate-800 bg-slate-950/50 p-2 text-[10px] text-slate-400">
      <p className="text-amber-200/80">{TERRAIN_CAUTION}</p>
      {fidelityContext && isFidelityCouplingOn(fidelityContext) && (
        <p className="text-amber-100/80">{BANNER_FIDELITY_TRUTH}</p>
      )}
      <p className="text-slate-500">
        Sandbox ENU + fictional terrain only — not synchronized to tactical UTC timelines.
      </p>
      {selectedEntity ? (
        <>
          {(() => {
            const x = Number(selectedEntity.pose.x ?? 0);
            const y = Number(selectedEntity.pose.y ?? 0);
            const z = Number(selectedEntity.pose.z ?? 0);
            const simAgl =
              fidelityContext && selectedEntity.entity_id
                ? simAglForEntity(fidelityContext, selectedEntity.entity_id)
                : null;
            const rel = entityTerrainRelation(x, y, z, simAgl);
            return (
              <>
                <dl className="grid gap-0.5">
                  <div>
                    <dt className="inline font-medium text-slate-500">terrain_m: </dt>
                    <dd className="inline font-mono">{rel.terrain_m.toFixed(1)}</dd>
                  </div>
                  <div>
                    <dt className="inline font-medium text-slate-500">registry_z_m: </dt>
                    <dd className="inline font-mono">{rel.registry_z_m.toFixed(1)}</dd>
                  </div>
                  <div>
                    <dt className="inline font-medium text-slate-500">
                      display_agl_m (explanatory):{" "}
                    </dt>
                    <dd className="inline font-mono">{rel.display_agl_m.toFixed(1)}</dd>
                  </div>
                  {rel.sim_agl_m != null && (
                    <div>
                      <dt className="inline font-medium text-violet-300/90">
                        sim_agl_m (truth_attested):{" "}
                      </dt>
                      <dd className="inline font-mono">{rel.sim_agl_m.toFixed(1)}</dd>
                    </div>
                  )}
                  <div>
                    <dt className="inline font-medium text-slate-500">nearest ridge: </dt>
                    <dd className="inline">{rel.nearest_ridge ?? "—"}</dd>
                  </div>
                  <div>
                    <dt className="inline font-medium text-slate-500">contour band: </dt>
                    <dd className="inline">
                      {rel.contour_level_m != null ? `≥${rel.contour_level_m}m` : "—"}
                    </dd>
                  </div>
                </dl>
                <p className="text-slate-500">{terrainContextLine(rel)}</p>
              </>
            );
          })()}
          <p>
            <span className="font-medium text-slate-500">visibility hint: </span>
            {visibilityHint(selectedEntity, entities)}
          </p>
          {(() => {
            const occ = nearestOcclusionTarget(selectedEntity, entities);
            if (!occ) return null;
            return (
              <>
                <p>
                  <span className="font-medium text-slate-500">occlusion (heuristic): </span>
                  {occ.status} → {occ.target.entity_type} · {occ.target.entity_id.slice(0, 8)}
                </p>
                <p className="text-slate-500">{losCueSummary(selectedEntity, occ.target)}</p>
              </>
            );
          })()}
          {selectedEntity.entity_type === "radar" &&
            (() => {
              const ctx = sensorDomeContext(selectedEntity, entities);
              return (
                <p>
                  <span className="font-medium text-slate-500">sensor dome: </span>
                  {ctx.context_label} — {ctx.note}
                </p>
              );
            })()}
        </>
      ) : (
        <p>Select an entity for terrain relation and occlusion context.</p>
      )}
    </div>
  );
}
