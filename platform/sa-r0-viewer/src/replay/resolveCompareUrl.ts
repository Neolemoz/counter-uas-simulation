import { loadBundleFromUrl } from "./loadBundle";
import { loadScenarioCatalog, demoUrlFromPackId } from "./loadCatalog";
import { loadComparePairs, pairById } from "./loadComparePairs";
import { useCompareStore } from "./compareStore";

export async function tryResolveCompareFromUrl(): Promise<boolean> {
  const params = new URLSearchParams(window.location.search);
  const pairId = params.get("pair");
  const compare = params.get("compare");

  if (pairId) {
    const pairs = await loadComparePairs();
    const pair = pairById(pairs, pairId);
    if (!pair) return false;
    const [a, b] = await Promise.all([
      loadBundleFromUrl(pair.slot_a.demo_bundle_url),
      loadBundleFromUrl(pair.slot_b.demo_bundle_url),
    ]);
    useCompareStore.getState().enterCompare(a, b, pairId);
    return true;
  }

  if (compare) {
    const [packA, packB] = compare.split(",").map((s) => s.trim());
    if (!packA || !packB) return false;
    const catalog = await loadScenarioCatalog();
    const urlA = demoUrlFromPackId(catalog, packA);
    const urlB = demoUrlFromPackId(catalog, packB);
    if (!urlA || !urlB) return false;
    const [a, b] = await Promise.all([loadBundleFromUrl(urlA), loadBundleFromUrl(urlB)]);
    useCompareStore.getState().enterCompare(a, b, null);
    return true;
  }

  return false;
}
