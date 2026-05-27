export function sortSlotsBySessionOrder<T extends { sessionId: string }>(
  slots: T[],
  orderedSessionIds: readonly string[],
): T[] {
  const rank = new Map(orderedSessionIds.map((id, i) => [id, i]));
  return [...slots].sort((a, b) => {
    const ra = rank.get(a.sessionId) ?? Number.MAX_SAFE_INTEGER;
    const rb = rank.get(b.sessionId) ?? Number.MAX_SAFE_INTEGER;
    if (ra !== rb) return ra - rb;
    return a.sessionId.localeCompare(b.sessionId);
  });
}
