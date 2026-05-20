import { describe, expect, it } from "vitest";
import { replayStoryboardSchema } from "./presentationSchema";

describe("presentationSchema", () => {
  it("parses storyboard fixture shape", () => {
    const data = {
      artifact_type: "replay_storyboard_v1",
      schema_version: "replay_storyboard_v1",
      storyboard_id: "test_deck",
      title: "Test deck",
      estimated_minutes: 15,
      governance: { notice: "Derived replay presentation artifact." },
      scenes: [
        {
          scene_id: "s1",
          label: "Scene 1",
          target_url: "?demo=valley_ingress&chapter=0",
          copy: "Review ingress.",
          importance_tags: ["topology"],
        },
      ],
    };
    const parsed = replayStoryboardSchema.parse(data);
    expect(parsed.storyboard_id).toBe("test_deck");
  });
});
