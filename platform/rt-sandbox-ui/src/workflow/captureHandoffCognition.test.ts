import { describe, expect, it } from "vitest";
import {
  captureReadinessFromLifecycle,
  CAPTURE_PIPELINE_STEPS,
} from "./captureHandoffCognition";

describe("captureReadinessFromLifecycle", () => {
  it("returns neutral when disconnected", () => {
    const r = captureReadinessFromLifecycle("running", false);
    expect(r.tone).toBe("neutral");
    expect(r.label).toContain("No active session");
  });

  it("marks stopped as capture eligible", () => {
    const r = captureReadinessFromLifecycle("stopped", true);
    expect(r.tone).toBe("ok");
    expect(r.label).toBe("Capture eligible");
  });

  it("marks captured after capture", () => {
    const r = captureReadinessFromLifecycle("captured", true);
    expect(r.label).toBe("Session captured");
  });

  it("warns when running", () => {
    const r = captureReadinessFromLifecycle("running", true);
    expect(r.tone).toBe("warn");
  });

  it("errors on failed states", () => {
    expect(captureReadinessFromLifecycle("failed", true).tone).toBe("error");
  });
});

describe("CAPTURE_PIPELINE_STEPS", () => {
  it("defines maintainer pipeline phases", () => {
    expect(CAPTURE_PIPELINE_STEPS.length).toBeGreaterThanOrEqual(5);
    expect(CAPTURE_PIPELINE_STEPS[0].cli).toContain("capture_session");
  });
});
