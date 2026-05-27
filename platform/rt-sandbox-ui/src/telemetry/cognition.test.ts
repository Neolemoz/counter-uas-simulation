import { describe, expect, it } from "vitest";
import {
  formatAuthorityChip,
  sessionContextLine,
} from "@/telemetry/cognition";

describe("telemetry cognition V1", () => {
  it("formats authority chips", () => {
    expect(formatAuthorityChip("command_authoritative")).toBe("cmd auth");
    expect(formatAuthorityChip("explanatory_sync")).toBe("sync mirror");
  });

  it("builds session context line", () => {
    const line = sessionContextLine("abcdefgh-9999", "background");
    expect(line).toContain("abcdefgh");
    expect(line).toContain("background");
    expect(line).toContain("correlation");
  });
});
