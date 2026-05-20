import { z } from "zod";

const catalogPackSchema = z.object({
  pack_id: z.string(),
  scenario_id: z.string(),
  title: z.string(),
  pack_path: z.string(),
  topology_tags: z.array(z.string()),
  replay_tags: z.array(z.string()).optional(),
  ingress_archetype: z.string().optional(),
  overlay_descriptors: z
    .array(z.object({ kind: z.string(), count: z.number().optional(), label: z.string().optional() }))
    .optional(),
  ambiguity_profile: z.object({ level: z.string(), focus_tags: z.array(z.string()).optional() }).optional(),
  replay_duration_class: z.string().optional(),
  terrain_profile: z.string().optional(),
  narrative_focus: z.array(z.string()).optional(),
  category: z.string().optional(),
  demo_bundle: z.string(),
  demo_bundle_url: z.string(),
  baseline_pack_id: z.string().optional(),
  compare_pair_ids: z.array(z.string()).optional(),
});

export const scenarioCatalogSchema = z.object({
  artifact_type: z.literal("scenario_topology_catalog"),
  schema_version: z.literal("scenario_topology_catalog_v1"),
  governance: z.object({ notice: z.string() }).optional(),
  packs: z.array(catalogPackSchema),
});

export type ScenarioCatalog = z.infer<typeof scenarioCatalogSchema>;
export type CatalogPack = z.infer<typeof catalogPackSchema>;

export const CATALOG_URL = "/demo/catalog.json";

export const CATEGORY_LABELS: Record<string, string> = {
  terrain_masking: "Terrain masking",
  ingress_geometry: "Ingress geometry",
  multi_threat: "Multi-threat",
  detection_timing: "Detection timing",
  corridor_defense: "Corridor defense",
  topology_experiment: "Topology experiments",
};

export const COMPARE_MODE_LABELS: Record<string, string> = {
  replay_ab: "Replay A/B",
  topology_ab: "Topology A/B",
  sensor_study: "Sensor placement study",
};

export const comparePairsSchema = z.object({
  artifact_type: z.literal("scenario_compare_pairs_v1"),
  schema_version: z.literal("scenario_compare_pairs_v1"),
  governance: z.object({ notice: z.string(), anti_claims: z.array(z.string()).optional() }).optional(),
  pairs: z.array(
    z.object({
      pair_id: z.string(),
      label: z.string(),
      mode: z.enum(["replay_ab", "topology_ab", "sensor_study"]),
      slot_a: z.object({ pack_id: z.string(), demo_bundle_url: z.string() }),
      slot_b: z.object({ pack_id: z.string(), demo_bundle_url: z.string() }),
      shared_log_ref: z.string().optional(),
      governance_notice: z.string(),
    }),
  ),
});

export type ComparePairsManifest = z.infer<typeof comparePairsSchema>;
export const COMPARE_PAIRS_URL = "/demo/compare_pairs.json";
