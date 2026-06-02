/** UI enablement for scenario controls (mirrors apply_scenario lifecycle allow-list). */

export type ScenarioControlStates = {
  applyScenario: boolean;
};

export function scenarioControlStates(sessionState: string): ScenarioControlStates {
  const running = sessionState === "running";
  const paused = sessionState === "paused";
  return {
    applyScenario: running || paused,
  };
}

export const SCENARIO_CONTROL_GOVERNANCE =
  "RT-local scenario apply only — pushes the current editor layout into this session. " +
  "Not SA import, replay authority, or corpus promotion.";
