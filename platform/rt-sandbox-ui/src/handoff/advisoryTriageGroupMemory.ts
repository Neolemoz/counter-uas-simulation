import type { TriageGroupMode } from "./advisoryTypes";

const STORAGE_KEY = "rt_advisory_triage_group_open_v1";

type OpenMap = Record<string, boolean>;

function storageKey(mode: TriageGroupMode, groupKey: string): string {
  return `${mode}:${groupKey}`;
}

function readMap(): OpenMap {
  if (typeof localStorage === "undefined") return {};
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return {};
    return JSON.parse(raw) as OpenMap;
  } catch {
    return {};
  }
}

function writeMap(map: OpenMap): void {
  if (typeof localStorage === "undefined") return;
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(map));
  } catch {
    /* ignore quota */
  }
}

export function getGroupOpenState(
  mode: TriageGroupMode,
  groupKey: string,
  defaultOpen: boolean,
): boolean {
  const map = readMap();
  const key = storageKey(mode, groupKey);
  if (key in map) return map[key] ?? defaultOpen;
  return defaultOpen;
}

export function setGroupOpenState(
  mode: TriageGroupMode,
  groupKey: string,
  open: boolean,
): void {
  const map = readMap();
  map[storageKey(mode, groupKey)] = open;
  writeMap(map);
}
