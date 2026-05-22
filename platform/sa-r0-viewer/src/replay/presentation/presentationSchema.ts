import { z } from "zod";

export const replayStoryboardSchema = z.object({
  artifact_type: z.literal("replay_storyboard_v1"),
  schema_version: z.literal("replay_storyboard_v1"),
  storyboard_id: z.string(),
  title: z.string(),
  estimated_minutes: z.number(),
  governance: z.object({
    notice: z.string(),
    anti_claims: z.array(z.string()).optional(),
  }),
  scenes: z.array(
    z.object({
      scene_id: z.string(),
      label: z.string(),
      target_url: z.string(),
      copy: z.string(),
      chapter: z.number().optional(),
      importance_tags: z
        .array(z.enum(["ambiguity", "los", "topology", "assignment", "pacing"]))
        .optional(),
    }),
  ),
});

export type ReplayStoryboard = z.infer<typeof replayStoryboardSchema>;

export const storyboardIndexSchema = z.object({
  artifact_type: z.literal("replay_storyboard_index_v1"),
  schema_version: z.literal("replay_storyboard_index_v1"),
  governance: z.object({ notice: z.string() }).optional(),
  storyboards: z.array(
    z.object({
      storyboard_id: z.string(),
      title: z.string(),
      estimated_minutes: z.number().optional(),
      storyboard_url: z.string(),
    }),
  ),
});

export type StoryboardIndex = z.infer<typeof storyboardIndexSchema>;

export const PRESENTATIONS_INDEX_URL = "/demo/presentations/index.json";
