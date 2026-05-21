import { z } from "zod";

const sampleSchema = z.object({
  t: z.number(),
  x_m: z.number(),
  y_m: z.number(),
  z_m: z.number().optional(),
  source: z.string().optional(),
});

const trackSchema = z.object({
  track_id: z.string(),
  role: z.string(),
  samples: z.array(sampleSchema),
  style: z
    .object({
      polyline: z.string().optional(),
      authoritative: z.boolean().optional(),
    })
    .optional(),
  interpretation_caveat: z.string().optional(),
});

const entitySchema = z.object({
  entity_id: z.string(),
  kind: z.string(),
  position_enu_m: z.tuple([z.number(), z.number(), z.number()]),
  label: z.string(),
  authoritative: z.boolean().optional(),
});

const zoneSchema = z.object({
  zone_id: z.string(),
  kind: z.string(),
  display_label: z.string().optional(),
  geometry: z.object({
    type: z.string(),
    center_enu_m: z.array(z.number()).optional(),
    radius_m: z.number().optional(),
    vertices_enu_m: z.array(z.array(z.number())).optional(),
  }),
  caveat: z.string().optional(),
});

const overlaySchema = z.object({
  overlay_id: z.string(),
  kind: z.string(),
  geometry: z.object({
    type: z.string(),
    vertices_enu_m: z.array(z.array(z.number())).optional(),
    ridge_outline_enu_m: z.array(z.array(z.number())).optional(),
  }),
  caveat: z.string().optional(),
  linked_event_ids: z.array(z.string()).optional(),
  active_t_range: z.tuple([z.number(), z.number()]).optional(),
});

const losSegmentSchema = z.object({
  segment_id: z.string(),
  from_entity_id: z.string(),
  from_kind: z.string().optional(),
  to_track_id: z.string(),
  status: z.enum(["visible", "partially_occluded", "terrain_blocked"]),
  polyline_enu_m: z.array(z.tuple([z.number(), z.number(), z.number()])),
  caveat: z.string(),
  t: z.number().optional(),
  linked_event_ids: z.array(z.string()).optional(),
});

const terrainModelSchema = z.object({
  type: z.literal("fictional_heightmap"),
  grid_enu_m: z.object({
    origin: z.array(z.number()),
    spacing_m: z.number(),
    size: z.number(),
    heights_m: z.array(z.array(z.number())),
  }),
  caveat: z.string().optional(),
});

const overlayDescriptorSchema = z.object({
  kind: z.string(),
  count: z.number().optional(),
  label: z.string().optional(),
});

const ambiguityProfileSchema = z.object({
  level: z.string(),
  focus_tags: z.array(z.string()).optional(),
});

const provenanceSchema = z.object({
  fixture_source: z.string(),
  fictional_disclaimer: z.string(),
});

export const replaySaBundleSchema = z.object({
  artifact_type: z.literal("replay_sa_bundle"),
  bundle_schema_version: z.literal("replay_sa_bundle_v1"),
  mode: z.literal("replay_static"),
  governance: z.object({
    notice: z.string(),
    constraints: z.array(z.string()).optional(),
    anti_claims: z.array(z.string()).optional(),
  }),
  lineage: z.record(z.unknown()),
  scenario: z.object({
    scenario_id: z.string(),
    title: z.string(),
    topology_tags: z.array(z.string()).optional(),
    replay_tags: z.array(z.string()).optional(),
    ingress_archetype: z.string().optional(),
    overlay_descriptors: z.array(overlayDescriptorSchema).optional(),
    ambiguity_profile: ambiguityProfileSchema.optional(),
    replay_duration_class: z.string().optional(),
    terrain_profile: z.string().optional(),
    narrative_focus: z.array(z.string()).optional(),
    provenance: provenanceSchema.optional(),
    scenario_pack_id: z.string().optional(),
    catalog_pack_id: z.string().optional(),
    launch_geometry: z.record(z.number()).optional(),
    terrain_model: terrainModelSchema.optional(),
  }),
  comparison_hints: z
    .object({
      topology_key: z.string(),
      scenario_id: z.string(),
      catalog_entry_id: z.string(),
      duration_class: z.string().optional(),
      seed: z.unknown().optional(),
      comparison_ready: z.boolean().optional(),
      sensor_layout_id: z.string().optional(),
      compare_mode: z.enum(["replay_ab", "topology_ab", "sensor_study"]).optional(),
      baseline_topology_key: z.string().optional(),
      paired_topology_key: z.string().optional(),
      sweep_id: z.string().optional(),
      member_index: z.number().optional(),
      sweep_variant_id: z.string().optional(),
    })
    .optional(),
  spatial_analytics: z
    .object({
      grid: z.object({
        origin_enu_m: z.array(z.number()),
        spacing_m: z.number(),
        size: z.array(z.number()),
      }),
      layers: z.record(z.unknown()),
    })
    .optional(),
  source_artifacts: z
    .object({
      scenario_pack: z.string().optional(),
      embedded: z.boolean().optional(),
    })
    .passthrough()
    .optional(),
  georef_display: z.object({
    frame: z.string(),
    origin_enu_m: z.array(z.number()),
    anchor: z.object({
      lat_deg: z.number(),
      lon_deg: z.number(),
      h_m: z.number(),
    }),
    caveat: z.string().optional(),
  }),
  clock: z.object({
    domain: z.string(),
    duration: z.object({
      start: z.number(),
      end: z.number(),
      step: z.number(),
    }),
    markers: z.array(
      z.object({
        t: z.number(),
        event_id: z.string().optional(),
        label: z.string().optional(),
        category: z.string().optional(),
      }),
    ),
  }),
  tracks: z.array(trackSchema),
  entities_static: z.array(entitySchema),
  zones: z.array(zoneSchema),
  overlays: z.array(overlaySchema),
  los_segments: z.array(losSegmentSchema).optional(),
  narrative: z.object({
    events: z.array(z.record(z.unknown())),
    windows: z.array(z.record(z.unknown())).optional(),
    bookmarks: z.array(z.record(z.unknown())).optional(),
    annotations: z.array(z.record(z.unknown())).optional(),
  }),
  comprehension: z.object({
    scan_guide: z.array(z.string()).optional(),
    headline: z.string().optional().nullable(),
    at_a_glance: z.object({
      cards: z.array(
        z.object({
          label: z.string(),
          value: z.string(),
        }),
      ),
      summary: z.record(z.unknown()).optional(),
    }),
  }),
  views: z.record(z.unknown()).optional(),
  panels: z
    .object({
      telemetry_series: z.array(z.record(z.unknown())).optional(),
      threat_assessment: z.array(z.record(z.unknown())).optional(),
    })
    .optional(),
  interpretation_caveats: z.array(z.string()).optional(),
  corpus_ref: z
    .object({
      corpus_id: z.string(),
      entry_id: z.string(),
      lineage_parent_ids: z.array(z.string()).optional(),
      index_revision: z.string().optional(),
    })
    .optional(),
  presentation: z
    .object({
      walkthrough_id: z.string(),
      chapters: z.array(
        z.object({
          chapter_id: z.string(),
          title: z.string(),
          t_start: z.number(),
          t_end: z.number(),
          summary: z.string(),
          focus_event_ids: z.array(z.string()).optional(),
          spotlight_annotation_ids: z.array(z.string()).optional(),
          topology_highlight: z
            .enum(["ridge", "valley", "corridor", "assignment", "none"])
            .optional(),
          visible_layers: z.record(z.boolean()).optional(),
          los_scope: z.enum(["all", "selected_track"]).optional(),
          spatial_declutter: z.enum(["top_k", "threshold", "off"]).optional(),
          narrative_emphasis: z
            .enum(["ambiguity", "los", "topology", "assignment", "pacing"])
            .optional(),
        }),
      ),
    })
    .optional(),
});

export type ReplaySaBundle = z.infer<typeof replaySaBundleSchema>;
