import { describe, expect, it } from "vitest";
import { renderToStaticMarkup } from "react-dom/server";
import { LifecycleControlBar } from "./RefreshControls";

describe("LifecycleControlBar", () => {
  const handlers = {
    onPause: () => {},
    onResume: () => {},
    onReset: () => {},
    onStopSession: () => {},
  };

  it("disables pause when not running", () => {
    const markup = renderToStaticMarkup(
      <LifecycleControlBar
        connected
        editingEnabled
        busy={false}
        sessionState="paused"
        {...handlers}
      />,
    );
    const pauseButton = markup.match(/<button[^>]*>Pause<\/button>/)?.[0] ?? "";
    expect(pauseButton).toContain('disabled=""');
    const resumeButton = markup.match(/<button[^>]*>Resume<\/button>/)?.[0] ?? "";
    expect(resumeButton).not.toContain('disabled=""');
  });

  it("enables pause when running", () => {
    const markup = renderToStaticMarkup(
      <LifecycleControlBar
        connected
        editingEnabled
        busy={false}
        sessionState="running"
        {...handlers}
      />,
    );
    const pauseButton = markup.match(/<button[^>]*>Pause<\/button>/)?.[0] ?? "";
    expect(pauseButton).not.toContain('disabled=""');
    const resumeButton = markup.match(/<button[^>]*>Resume<\/button>/)?.[0] ?? "";
    expect(resumeButton).toContain('disabled=""');
  });
});
