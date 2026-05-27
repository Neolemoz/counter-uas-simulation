import { shortSessionId } from "@/workstation/sessionVisualIdentity";

const STORAGE_KEY = "rt_session_display_names_v1";
const MAX_LABEL_LENGTH = 48;

export type SessionDisplayNameMap = Record<string, string>;

function readMap(): SessionDisplayNameMap {
  if (typeof localStorage === "undefined") return {};
  const raw = localStorage.getItem(STORAGE_KEY);
  if (!raw) return {};
  try {
    const parsed = JSON.parse(raw) as unknown;
    if (!parsed || typeof parsed !== "object") return {};
    const out: SessionDisplayNameMap = {};
    for (const [key, value] of Object.entries(parsed)) {
      if (typeof value === "string" && value.trim()) {
        out[key] = value.trim().slice(0, MAX_LABEL_LENGTH);
      }
    }
    return out;
  } catch {
    return {};
  }
}

function writeMap(map: SessionDisplayNameMap): void {
  if (typeof localStorage === "undefined") return;
  localStorage.setItem(STORAGE_KEY, JSON.stringify(map, null, 2));
}

export function getSessionDisplayName(sessionId: string): string | null {
  return readMap()[sessionId] ?? null;
}

export function getSessionDisplayLabel(sessionId: string): string {
  return getSessionDisplayName(sessionId) ?? shortSessionId(sessionId);
}

export function setSessionDisplayName(sessionId: string, name: string): void {
  const trimmed = name.trim().slice(0, MAX_LABEL_LENGTH);
  const map = readMap();
  if (!trimmed) {
    delete map[sessionId];
  } else {
    map[sessionId] = trimmed;
  }
  writeMap(map);
}

export function clearSessionDisplayName(sessionId: string): void {
  const map = readMap();
  delete map[sessionId];
  writeMap(map);
}

export function readSessionDisplayNameMap(): SessionDisplayNameMap {
  return readMap();
}
