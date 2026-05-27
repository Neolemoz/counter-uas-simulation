import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { TacticalAutonomousPanel } from "./TacticalAutonomousPanel";

describe("TacticalAutonomousPanel", () => {
  it("shows pause resume and return to manual", () => {
    const markup = renderToStaticMarkup(
      <TacticalAutonomousPanel
        sessionId="sess-1"
        editingEnabled
        entities={[]}
        state={{
          tactical_mode: "autonomous",
          autonomous_loop_status: "running",
          assignment_lock_active: false,
          authority_label: "tactical_controller_authoritative",
        }}
        busy={false}
        error={null}
        onPause={() => undefined}
        onResume={() => undefined}
        onReturnToManual={() => undefined}
      />,
    );
    expect(markup).toContain("Pause loop");
    expect(markup).toContain("Resume loop");
    expect(markup).toContain("Return to Manual");
    expect(markup).not.toContain("engage");
  });
});
