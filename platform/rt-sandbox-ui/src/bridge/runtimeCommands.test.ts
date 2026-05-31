import { describe, expect, it, vi, beforeEach } from "vitest";
import {
  assignTarget,
  cancelAssignment,
  pauseSim,
  resumeSim,
  spawnDefender,
} from "./runtimeCommands";

describe("runtimeCommands", () => {
  beforeEach(() => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });
  });

  it("pause_sim payload shape", async () => {
    await pauseSim("sid");
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("pause_sim");
    expect(body.session_id).toBe("sid");
  });

  it("resume_sim payload shape", async () => {
    await resumeSim("sid");
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("resume_sim");
    expect(body.session_id).toBe("sid");
  });

  it("spawn_defender payload shape", async () => {
    await spawnDefender("sid", { pose: { x: 1, y: 2, z: 10, yaw_deg: 0 } });
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("spawn_defender");
    expect(body.session_id).toBe("sid");
    expect(body.payload.pose.x).toBe(1);
  });

  it("spawn_defender without pose", async () => {
    await spawnDefender("sid");
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("spawn_defender");
    expect(body.payload).toEqual({});
  });

  it("assign_target payload shape", async () => {
    await assignTarget("sid", { defender_id: "d1", target_id: "t1" });
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("assign_target");
    expect(body.payload.defender_id).toBe("d1");
    expect(body.payload.target_id).toBe("t1");
  });

  it("cancel_assignment payload shape", async () => {
    await cancelAssignment("sid", { defender_id: "d1" });
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("cancel_assignment");
    expect(body.payload.defender_id).toBe("d1");
  });
});
