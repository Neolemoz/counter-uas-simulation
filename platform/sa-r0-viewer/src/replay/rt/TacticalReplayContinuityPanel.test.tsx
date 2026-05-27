import React from "react";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { describe, expect, it } from "vitest";
import { renderToString } from "react-dom/server";
import { parseBundleJson } from "../loadBundle";
import { TacticalReplayContinuityPanel } from "./TacticalReplayContinuityPanel";

const repoRoot = join(dirname(fileURLToPath(import.meta.url)), "../../../../..");
const demoPath = join(
  repoRoot,
  "platform/sa-r0-viewer/public/demo/rt_tactical_continuity/index.json",
);
const ridgeDemoPath = join(repoRoot, "fixtures/sa_r0/demo_ridge_defense/index.json");

describe("TacticalReplayContinuityPanel", () => {
  it("renders governance banner and timelines without assign controls", () => {
    const bundle = parseBundleJson(readFileSync(demoPath, "utf-8"));
    const html = renderToString(<TacticalReplayContinuityPanel bundle={bundle} />);
    expect(html).toContain("explanatory replay only");
    expect(html).toContain("Mode switches");
    expect(html).toContain("manual");
    expect(html.toLowerCase()).not.toContain("assign target");
    expect(html.toLowerCase()).not.toContain("engage");
  });

  it("shows empty state when continuity absent", () => {
    const bundle = parseBundleJson(readFileSync(ridgeDemoPath, "utf-8"));
    expect(bundle.rt_tactical_replay_continuity).toBeUndefined();
    const html = renderToString(<TacticalReplayContinuityPanel bundle={bundle} />);
    expect(html).toContain("No RT tactical continuity");
  });
});
