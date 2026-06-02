/** UI-only Monte Carlo run-count presets (no execution in RT sandbox). */

export type ScenarioEvaluationPresetId = "quick" | "standard" | "deep";

export type ScenarioEvaluationPreset = {
  id: ScenarioEvaluationPresetId;
  label: string;
  runs: number;
  description: string;
};

export const SCENARIO_EVALUATION_PRESETS: ScenarioEvaluationPreset[] = [
  {
    id: "quick",
    label: "Quick",
    runs: 10,
    description: "10 runs — preview sizing only",
  },
  {
    id: "standard",
    label: "Standard",
    runs: 50,
    description: "50 runs — preview sizing only",
  },
  {
    id: "deep",
    label: "Deep",
    runs: 100,
    description: "100 runs — preview sizing only",
  },
];

export function presetById(id: ScenarioEvaluationPresetId): ScenarioEvaluationPreset {
  const row = SCENARIO_EVALUATION_PRESETS.find((p) => p.id === id);
  return row ?? SCENARIO_EVALUATION_PRESETS[1];
}
