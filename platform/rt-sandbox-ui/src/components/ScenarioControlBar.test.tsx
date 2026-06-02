import { describe, expect, it, vi } from "vitest";
import { renderToStaticMarkup } from "react-dom/server";
import { applyScenario } from "@/bridge/scenarioCommands";
import { SCENARIO_CONTROL_GOVERNANCE } from "@/scenario/scenarioControlStates";
import { ScenarioControlBar } from "./RefreshControls";

describe("ScenarioControlBar", () => {
  it("renders nothing when not connected", () => {
    const markup = renderToStaticMarkup(
      <ScenarioControlBar
        connected={false}
        applyDisabled
        entityCount={2}
        onApplyScenario={() => {}}
      />,
    );
    expect(markup).toBe("");
  });

  it("renders governance and apply control when connected", () => {
    const markup = renderToStaticMarkup(
      <ScenarioControlBar
        connected
        applyDisabled={false}
        entityCount={2}
        onApplyScenario={() => {}}
      />,
    );
    expect(markup).toContain('data-testid="scenario-control-bar"');
    expect(markup).toContain(SCENARIO_CONTROL_GOVERNANCE);
    expect(markup).toContain("Apply scenario");
  });

  it("disables apply when applyDisabled is true", () => {
    const markup = renderToStaticMarkup(
      <ScenarioControlBar
        connected
        applyDisabled
        entityCount={2}
        onApplyScenario={() => {}}
      />,
    );
    const applyButton = markup.match(/<button[^>]*data-testid="scenario-apply"[^>]*>/)?.[0] ?? "";
    expect(applyButton).toMatch(/\sdisabled(?:=""|(?=\s|>))/);
  });
});

describe("apply_scenario dispatch", () => {
  it("uses existing scenarioCommands helper", async () => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });

    await applyScenario("sid", {
      terrain_preset: "rt_sandbox_flat",
      assets: [],
      defenders: [],
      attackers: [],
    });
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("apply_scenario");
    expect(body.session_id).toBe("sid");
  });
});
