import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { afterEach, describe, expect, it, vi } from "vitest";
import { useCompareStore } from "./compareStore";
import { tryResolveCompareFromUrl } from "./resolveCompareUrl";

const repoRoot = join(dirname(fileURLToPath(import.meta.url)), "../../../..");
const comparePairsPath = join(repoRoot, "platform/sa-r0-viewer/public/demo/compare_pairs.json");
const runtimeBasePath = join(
  repoRoot,
  "platform/sa-r0-viewer/public/demo/rt_runtime_compare/base/index.json",
);
const runtimeVariantPath = join(
  repoRoot,
  "platform/sa-r0-viewer/public/demo/rt_runtime_compare/variant/index.json",
);

function jsonResponse(path: string): Response {
  return new Response(readFileSync(path, "utf-8"), {
    status: 200,
    headers: { "content-type": "application/json" },
  });
}

describe("tryResolveCompareFromUrl", () => {
  afterEach(() => {
    vi.restoreAllMocks();
    useCompareStore.getState().exitCompare();
    Reflect.deleteProperty(globalThis, "window");
  });

  it("loads runtime replay compare pair through existing compare path", async () => {
    vi.stubGlobal("window", { location: { search: "?pair=rt_runtime_capture_replay" } });
    vi.stubGlobal(
      "fetch",
      vi.fn(async (url: string | URL | Request) => {
        const href = String(url);
        if (href === "/demo/compare_pairs.json") return jsonResponse(comparePairsPath);
        if (href === "/demo/rt_runtime_compare/base/index.json") return jsonResponse(runtimeBasePath);
        if (href === "/demo/rt_runtime_compare/variant/index.json") return jsonResponse(runtimeVariantPath);
        return new Response("not found", { status: 404 });
      }),
    );

    await expect(tryResolveCompareFromUrl()).resolves.toBe(true);
    const state = useCompareStore.getState();
    expect(state.mode).toBe("compare");
    expect(state.activePairId).toBe("rt_runtime_capture_replay");
    expect(state.slotA.bundle?.lineage.capture_id).toBe("capture-golden-0001");
    expect(state.slotB.bundle?.lineage.capture_id).toBe("capture-golden-0001-variant");
  });
});
