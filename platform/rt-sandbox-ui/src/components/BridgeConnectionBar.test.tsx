import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { BridgeConnectionBar } from "./RefreshControls";

describe("BridgeConnectionBar runtime profile", () => {
  it("shows selector before connect and active profile after connect", () => {
    const disconnected = renderToStaticMarkup(
      <BridgeConnectionBar
        connected={false}
        busy={false}
        sessionRuntimeProfile="mock_adapter"
        onSessionRuntimeProfileChange={() => undefined}
        onConnect={() => undefined}
        onDisconnect={() => undefined}
      />,
    );
    expect(disconnected).toContain('data-testid="runtime-profile-selector"');
    expect(disconnected).not.toContain('data-testid="bridge-active-runtime-profile"');

    const connected = renderToStaticMarkup(
      <BridgeConnectionBar
        connected
        busy={false}
        sessionRuntimeProfile="mock_adapter"
        activeRequestedRuntimeProfile="mock_adapter"
        onSessionRuntimeProfileChange={() => undefined}
        onConnect={() => undefined}
        onDisconnect={() => undefined}
      />,
    );
    expect(connected).toContain('data-testid="bridge-active-runtime-profile"');
    expect(connected).toContain("Mock adapter");
  });

  it("shows live stop governance when active session is live", () => {
    const markup = renderToStaticMarkup(
      <BridgeConnectionBar
        connected
        busy={false}
        sessionRuntimeProfile="live"
        activeRequestedRuntimeProfile="live"
        onSessionRuntimeProfileChange={() => undefined}
        onConnect={() => undefined}
        onDisconnect={() => undefined}
      />,
    );
    expect(markup).toContain('data-testid="bridge-live-stop-copy"');
    expect(markup).toContain("Live stop terminates the Gazebo adapter.");
  });
});
