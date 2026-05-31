import { describe, expect, it } from "vitest";
import {
  formatCaptureStartedShort,
  parseCaptureStatusFromResponse,
  shortCaptureId,
} from "./captureSummary";

describe("captureSummary", () => {
  it("shortens long capture ids", () => {
    expect(shortCaptureId("cap-12345678-abcd-efgh")).toBe("cap-1234…");
    expect(shortCaptureId("short")).toBe("short");
    expect(shortCaptureId(null)).toBe("—");
  });

  it("formats started utc time short", () => {
    expect(formatCaptureStartedShort("2026-06-01T12:34:56+00:00")).toBe("12:34:56");
    expect(formatCaptureStartedShort(null)).toBe("—");
  });

  it("parses inactive status", () => {
    expect(
      parseCaptureStatusFromResponse({
        ok: true,
        capture_active: false,
        capture_status: "inactive",
        capture_id: null,
        frames_count: 0,
        entities_count: 2,
      }),
    ).toEqual({
      captureActive: false,
      captureStatus: "inactive",
      captureId: null,
      framesCount: 0,
      entitiesCount: 2,
      startedUtc: null,
    });
  });

  it("parses active status", () => {
    expect(
      parseCaptureStatusFromResponse({
        ok: true,
        capture_active: true,
        capture_status: "active",
        capture_id: "cap-12345678-abcd",
        started_utc: "2026-06-01T12:34:56+00:00",
        frames_count: 42,
        entities_count: 3,
      }),
    ).toEqual({
      captureActive: true,
      captureStatus: "active",
      captureId: "cap-12345678-abcd",
      framesCount: 42,
      entitiesCount: 3,
      startedUtc: "2026-06-01T12:34:56+00:00",
    });
  });
});
