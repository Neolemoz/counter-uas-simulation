/** Coerce legacy bundle shapes before Zod validation (evaluation-side only). */

export function normalizeBundleForSchema(data: unknown): unknown {
  if (!data || typeof data !== "object" || Array.isArray(data)) {
    return data;
  }
  const root = data as Record<string, unknown>;
  const comprehension = root.comprehension;
  if (!comprehension || typeof comprehension !== "object" || Array.isArray(comprehension)) {
    return data;
  }
  const comp = { ...(comprehension as Record<string, unknown>) };
  const raw = comp.at_a_glance;
  if (Array.isArray(raw)) {
    comp.at_a_glance = {
      cards: raw
        .filter((item): item is Record<string, unknown> => !!item && typeof item === "object")
        .map((item) => ({
          label: String(item.label ?? ""),
          value: String(item.value ?? ""),
        })),
      summary: {},
    };
  } else if (raw && typeof raw === "object" && !Array.isArray(raw)) {
    const glance = raw as Record<string, unknown>;
    if (!Array.isArray(glance.cards)) {
      const cards: { label: string; value: string }[] = [];
      const summary: Record<string, unknown> =
        typeof glance.summary === "object" && glance.summary && !Array.isArray(glance.summary)
          ? { ...(glance.summary as Record<string, unknown>) }
          : {};
      for (const [key, value] of Object.entries(glance)) {
        if (key === "cards" || key === "summary") continue;
        summary[key] = value;
      }
      comp.at_a_glance = { cards, summary };
    }
  }
  return { ...root, comprehension: comp };
}
