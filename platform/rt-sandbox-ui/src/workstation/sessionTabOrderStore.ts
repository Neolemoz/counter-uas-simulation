const STORAGE_KEY = "rt_session_tab_order_v1";

function readOrder(): string[] {
  if (typeof localStorage === "undefined") return [];
  const raw = localStorage.getItem(STORAGE_KEY);
  if (!raw) return [];
  try {
    const parsed = JSON.parse(raw) as unknown;
    if (!Array.isArray(parsed)) return [];
    return parsed.filter((id): id is string => typeof id === "string" && id.length > 0);
  } catch {
    return [];
  }
}

function writeOrder(ids: string[]): void {
  if (typeof localStorage === "undefined") return;
  localStorage.setItem(STORAGE_KEY, JSON.stringify(ids, null, 2));
}

/** Filter saved order to connected ids, then append new connected ids in connect order. */
export function mergeTabOrder(
  connectedIds: readonly string[],
  savedOrder: readonly string[] = readOrder(),
): string[] {
  const connectedSet = new Set(connectedIds);
  const merged: string[] = [];
  for (const id of savedOrder) {
    if (connectedSet.has(id) && !merged.includes(id)) {
      merged.push(id);
    }
  }
  for (const id of connectedIds) {
    if (!merged.includes(id)) {
      merged.push(id);
    }
  }
  return merged;
}

export function readTabOrder(): string[] {
  return readOrder();
}

export function writeTabOrder(ids: readonly string[]): void {
  writeOrder([...ids]);
}

export function removeSessionFromTabOrder(sessionId: string): void {
  const next = readOrder().filter((id) => id !== sessionId);
  writeOrder(next);
}

export function validateTabOrder(
  nextOrder: readonly string[],
  connectedIds: readonly string[],
): boolean {
  if (nextOrder.length !== connectedIds.length) return false;
  const connectedSet = new Set(connectedIds);
  const seen = new Set<string>();
  for (const id of nextOrder) {
    if (!connectedSet.has(id) || seen.has(id)) return false;
    seen.add(id);
  }
  return seen.size === connectedSet.size;
}
