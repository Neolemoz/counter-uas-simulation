import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { replayCorpusIndexSchema } from "./synthesisSchema";

const REPO = join(dirname(fileURLToPath(import.meta.url)), "../../../../..");

describe("replayCorpusIndexSchema", () => {
  it("parses committed corpus index fixture", () => {
    const raw = readFileSync(
      join(REPO, "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"),
      "utf-8",
    );
    const data = JSON.parse(raw) as unknown;
    const index = replayCorpusIndexSchema.parse(data);
    expect(index.corpus_id).toBe("sa_r0_corpus_r1");
    expect(index.entries.length).toBeGreaterThan(50);
  });
});
