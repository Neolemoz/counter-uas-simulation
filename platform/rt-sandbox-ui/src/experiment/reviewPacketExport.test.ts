import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import {
  copyReviewPacketJson,
  downloadReviewPacket,
  exportReviewPacketJson,
  suggestedReviewPacketFilename,
} from "./reviewPacketExport";
import type { ExperimentReviewPacket } from "./reviewPacketSchema";

const samplePacket: ExperimentReviewPacket = {
  schema: "rt_experiment_review_packet_v1",
  packet_id: "review-test-001",
  created_at_utc: "2026-05-28T14:30:00Z",
  governance_banner: "RT EXPERIMENT REVIEW PACKET — advisory export only; not SA import authority",
  scope: { cohort_id: null },
};

describe("reviewPacketExport", () => {
  beforeEach(() => {
    vi.stubGlobal(
      "URL",
      Object.assign(URL, {
        createObjectURL: vi.fn(() => "blob:test"),
        revokeObjectURL: vi.fn(),
      }),
    );
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("export json omits sections array", () => {
    const json = exportReviewPacketJson(samplePacket);
    const parsed = JSON.parse(json) as Record<string, unknown>;
    expect(parsed).not.toHaveProperty("sections");
  });

  it("suggests safe filename from packet_id", () => {
    expect(suggestedReviewPacketFilename(samplePacket)).toBe("review-test-001.json");
  });

  it("triggers download link", () => {
    const click = vi.fn();
    const anchor = { href: "", download: "", click };
    vi.stubGlobal("document", {
      createElement: vi.fn(() => anchor),
    });

    downloadReviewPacket(samplePacket);

    expect(URL.createObjectURL).toHaveBeenCalled();
    expect(click).toHaveBeenCalled();
  });

  it("copies packet json via clipboard", async () => {
    const writeText = vi.fn().mockResolvedValue(undefined);
    vi.stubGlobal("navigator", { clipboard: { writeText } });

    const result = await copyReviewPacketJson(samplePacket);
    expect(result.ok).toBe(true);
    expect(writeText).toHaveBeenCalled();
    const payload = writeText.mock.calls[0]![0] as string;
    expect(payload).toContain("rt_experiment_review_packet_v1");
  });
});
