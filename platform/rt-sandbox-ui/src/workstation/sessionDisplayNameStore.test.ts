import { beforeEach, describe, expect, it, vi } from "vitest";
import {
  clearSessionDisplayName,
  getSessionDisplayLabel,
  getSessionDisplayName,
  setSessionDisplayName,
} from "./sessionDisplayNameStore";

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

beforeEach(() => {
  storage.clear();
});

describe("sessionDisplayNameStore", () => {
  it("returns short id when no display name", () => {
    const sid = "abcd-1234-5678-efgh";
    expect(getSessionDisplayLabel(sid)).toBe("abcd-123");
  });

  it("stores and reads display name", () => {
    const sid = "sess-display-1";
    setSessionDisplayName(sid, "  Ridge A  ");
    expect(getSessionDisplayName(sid)).toBe("Ridge A");
    expect(getSessionDisplayLabel(sid)).toBe("Ridge A");
  });

  it("clears on empty set", () => {
    const sid = "sess-display-2";
    setSessionDisplayName(sid, "Temp");
    clearSessionDisplayName(sid);
    expect(getSessionDisplayName(sid)).toBeNull();
  });
});
