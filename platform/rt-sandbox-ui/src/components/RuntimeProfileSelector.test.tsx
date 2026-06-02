import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { RuntimeProfileSelector } from "./RuntimeProfileSelector";

describe("RuntimeProfileSelector", () => {
  it("defaults stub option in markup", () => {
    const markup = renderToStaticMarkup(
      <RuntimeProfileSelector value="stub" onChange={() => undefined} />,
    );
    expect(markup).toContain('data-testid="runtime-profile-option-stub"');
    expect(markup).toContain("Stub runtime");
  });

  it("shows mock governance copy", () => {
    const markup = renderToStaticMarkup(
      <RuntimeProfileSelector value="mock_adapter" onChange={() => undefined} />,
    );
    expect(markup).toContain('data-testid="runtime-profile-option-mock_adapter"');
    expect(markup).toContain("Not live Gazebo");
    expect(markup).toContain("Not ROS-backed");
  });

  it("does not expose live adapter option", () => {
    const markup = renderToStaticMarkup(
      <RuntimeProfileSelector value="stub" onChange={() => undefined} />,
    );
    expect(markup).not.toContain("live_adapter");
    expect(markup).not.toContain("Live adapter");
  });
});
