import { useMemo } from "react";
import { useCompareStore } from "../compareStore";
import { alignAnnotations } from "../annotationAlign";

export function AnnotationsComparePanel() {
  const slotA = useCompareStore((s) => s.slotA);
  const slotB = useCompareStore((s) => s.slotB);

  const rows = useMemo(() => {
    if (!slotA.bundle || !slotB.bundle) return [];
    return alignAnnotations(slotA.bundle, slotB.bundle);
  }, [slotA.bundle, slotB.bundle]);

  if (rows.length === 0) return null;

  return (
    <section className="max-h-48 overflow-y-auto rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-2 font-semibold text-slate-200">Annotation alignment</h2>
      <ul className="space-y-1 text-xs">
        {rows.map((r) => (
          <li
            key={r.id}
            className={
              r.status === "matched"
                ? "text-slate-300"
                : r.status === "a_only"
                  ? "text-amber-200/80"
                  : "text-cyan-200/80"
            }
          >
            <span className="uppercase text-slate-600">{r.status.replace("_", " ")}</span>{" "}
            {r.label}
          </li>
        ))}
      </ul>
    </section>
  );
}
