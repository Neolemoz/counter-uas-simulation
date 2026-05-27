import { beforeEach, describe, expect, it, vi } from "vitest";
import {
  mergeTabOrder,
  readTabOrder,
  removeSessionFromTabOrder,
  validateTabOrder,
  writeTabOrder,
} from "./sessionTabOrderStore";

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

describe("sessionTabOrderStore", () => {
  it("merges saved order with new connected ids at tail", () => {
    const merged = mergeTabOrder(
      ["c", "a", "b"],
      ["b", "a", "x"],
    );
    expect(merged).toEqual(["b", "a", "c"]);
  });

  it("persists and reloads order", () => {
    writeTabOrder(["s2", "s1"]);
    expect(readTabOrder()).toEqual(["s2", "s1"]);
  });

  it("removes disconnected session from stored order", () => {
    writeTabOrder(["s1", "s2", "s3"]);
    removeSessionFromTabOrder("s2");
    expect(readTabOrder()).toEqual(["s1", "s3"]);
  });

  it("validates reorder permutations", () => {
    const connected = ["a", "b"];
    expect(validateTabOrder(["b", "a"], connected)).toBe(true);
    expect(validateTabOrder(["a"], connected)).toBe(false);
    expect(validateTabOrder(["a", "a"], connected)).toBe(false);
  });
});
