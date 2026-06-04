import { beforeEach, describe, expect, it, vi } from "vitest";
import {
  buildStartSessionPayload,
  DEFAULT_SESSION_RUNTIME_PROFILE,
  isSessionRuntimeProfile,
  LIVE_GAZEBO_GOVERNANCE,
  readStoredSessionRuntimeProfile,
  SESSION_RUNTIME_PROFILE_STORAGE_KEY,
  writeStoredSessionRuntimeProfile,
} from "./sessionRuntimeProfile";

const storage = new Map<string, string>();

vi.stubGlobal("localStorage", {
  getItem: (key: string) => storage.get(key) ?? null,
  setItem: (key: string, value: string) => {
    storage.set(key, value);
  },
  removeItem: (key: string) => {
    storage.delete(key);
  },
  clear: () => {
    storage.clear();
  },
});

beforeEach(() => {
  storage.clear();
});

describe("sessionRuntimeProfile", () => {
  it("defaults to stub runtime profile", () => {
    expect(readStoredSessionRuntimeProfile()).toBe(DEFAULT_SESSION_RUNTIME_PROFILE);
    expect(DEFAULT_SESSION_RUNTIME_PROFILE).toBe("stub");
  });

  it("persists mock selection in localStorage", () => {
    writeStoredSessionRuntimeProfile("mock_adapter");
    expect(storage.get(SESSION_RUNTIME_PROFILE_STORAGE_KEY)).toBe("mock_adapter");
    expect(readStoredSessionRuntimeProfile()).toBe("mock_adapter");
  });

  it("accepts live profile as session runtime profile", () => {
    expect(isSessionRuntimeProfile("live")).toBe(true);
    expect(isSessionRuntimeProfile("live_adapter")).toBe(false);
    writeStoredSessionRuntimeProfile("live");
    expect(readStoredSessionRuntimeProfile()).toBe("live");
  });

  it("omits payload for stub start_session", () => {
    expect(buildStartSessionPayload("stub")).toBeUndefined();
  });

  it("builds mock_adapter start_session payload", () => {
    expect(buildStartSessionPayload("mock_adapter")).toEqual({
      runtime_profile: "mock_adapter",
    });
  });

  it("builds live start_session payload", () => {
    expect(buildStartSessionPayload("live")).toEqual({
      runtime_profile: "live",
    });
    expect(LIVE_GAZEBO_GOVERNANCE).toContain("experimental");
  });
});
