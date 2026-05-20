import { useClockStore } from "@/replay/clockStore";

export function ThreatAssessmentPanel() {
  const bundle = useClockStore((s) => s.bundle);
  if (!bundle) return null;

  const items = bundle.panels?.threat_assessment ?? [];
  const glance = bundle.comprehension.at_a_glance;
  const dClassRaw = glance.summary?.selection_oracle_divergence_class;
  const dClassCard = glance.cards.find((c) => c.label.includes("Taxonomy ID"));

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-2 text-xs">
      <h2 className="mb-1 font-semibold text-slate-200">Threat assessment (replay-local)</h2>
      <p className="mb-2 text-[10px] text-slate-500">Taxonomy labels only — not threat scores.</p>
      <ul className="space-y-1 text-slate-400">
        {items.map((item, i) => (
          <li key={i}>
            {String(item.label)}
            <span className="block text-[10px] text-slate-600">{String(item.caveat)}</span>
          </li>
        ))}
        {dClassRaw != null && <li>D-class (summary): {String(dClassRaw)}</li>}
        {dClassCard && <li>{dClassCard.label}: {dClassCard.value}</li>}
      </ul>
    </section>
  );
}
