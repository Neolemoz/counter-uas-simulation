import { afterEach, describe, expect, it, vi } from "vitest";
import {
  clearSessionLayerVisibility,
  readSessionLayerVisibility,
  writeSessionLayerVisibility,
} from "./sessionLayerVisibilityStore";
import { defaultVisibilityFromRegistry } from "@/cesium/visualLayerRegistry";

const storage = new Map<string, string>();

vi.stubGlobal("localStorage", {
  getItem: (key: string) => storage.get(key) ?? null,
  setItem: (key: string, value: string) => {
    storage.set(key, value);
  },
  removeItem: (key: string) => {
    storage.delete(key);
  },
});

afterEach(() => {
  storage.clear();
});

describe("sessionLayerVisibilityStore", () => {
  it("returns null when session has no saved visibility", () => {
    expect(readSessionLayerVisibility("sess-a")).toBeNull();
  });

  it("round-trips visibility per session", () => {
    const vis = { ...defaultVisibilityFromRegistry(), showContourOverlays: true };
    writeSessionLayerVisibility("sess-a", vis);
    expect(readSessionLayerVisibility("sess-a")).toEqual(vis);
    expect(readSessionLayerVisibility("sess-b")).toBeNull();
  });

  it("clears session entry on disconnect", () => {
    writeSessionLayerVisibility("sess-a", defaultVisibilityFromRegistry());
    clearSessionLayerVisibility("sess-a");
    expect(readSessionLayerVisibility("sess-a")).toBeNull();
  });
});
