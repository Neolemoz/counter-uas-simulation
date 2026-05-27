import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { TacticalManualPanel } from "./TacticalManualPanel";

describe("TacticalManualPanel", () => {
  it("enables assisted and autonomous mode radios", () => {
    const markup = renderToStaticMarkup(
      <TacticalManualPanel
        sessionId="sess-1"
        editingEnabled
        entities={[]}
        selectedEntityId={null}
        mode="manual"
        state={{ tactical_mode: "manual" }}
        busy={false}
        error={null}
        targetPickActive={false}
        onModeChange={() => undefined}
        onUseSelectedInterceptor={() => undefined}
        onStartTargetPick={() => undefined}
        onAssign={() => undefined}
        onClear={() => undefined}
      />,
    );
    expect(markup).toContain("Assisted");
    expect(markup).toContain("Autonomous");
    expect(markup).toContain("Autonomous");
    expect(markup).not.toContain("engage");
    expect(markup).not.toMatch(/\bstrike\b/i);
  });

  it("shows assign candidate button", () => {
    const markup = renderToStaticMarkup(
      <TacticalManualPanel
        sessionId="sess-1"
        editingEnabled
        entities={[
          {
            entity_id: "i1",
            entity_type: "interceptor",
            pose: { x: 0, y: 0, z: 10 },
          },
        ]}
        selectedEntityId="i1"
        mode="manual"
        state={{
          selected_interceptor_id: "i1",
          selected_target_id: "t1",
        }}
        busy={false}
        error={null}
        targetPickActive={false}
        onModeChange={() => undefined}
        onUseSelectedInterceptor={() => undefined}
        onStartTargetPick={() => undefined}
        onAssign={() => undefined}
        onClear={() => undefined}
      />,
    );
    expect(markup).toContain("Assign candidate");
  });
});
