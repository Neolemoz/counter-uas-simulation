import type { VisualLayerVisibility } from "@/cesium/visualLayerRegistry";

const STORAGE_KEY = "rt_session_layer_visibility_v1";

type StoredMap = Record<string, VisualLayerVisibility>;

function readMap(): StoredMap {
  if (typeof localStorage === "undefined") return {};
  const raw = localStorage.getItem(STORAGE_KEY);
  if (!raw) return {};
  try {
    const parsed = JSON.parse(raw) as unknown;
    if (parsed == null || typeof parsed !== "object" || Array.isArray(parsed)) {
      return {};
    }
    return parsed as StoredMap;
  } catch {
    return {};
  }
}

function writeMap(map: StoredMap): void {
  if (typeof localStorage === "undefined") return;
  localStorage.setItem(STORAGE_KEY, JSON.stringify(map, null, 2));
}

export function readSessionLayerVisibility(
  sessionId: string,
): VisualLayerVisibility | null {
  const map = readMap();
  const entry = map[sessionId];
  return entry ?? null;
}

export function writeSessionLayerVisibility(
  sessionId: string,
  visibility: VisualLayerVisibility,
): void {
  const map = readMap();
  map[sessionId] = visibility;
  writeMap(map);
}

export function clearSessionLayerVisibility(sessionId: string): void {
  const map = readMap();
  if (!(sessionId in map)) return;
  delete map[sessionId];
  writeMap(map);
}
