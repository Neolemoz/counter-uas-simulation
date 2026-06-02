import { scenarioControlStates } from "@/scenario/scenarioControlStates";

/** Default ENU placement for rail spawn shortcuts (grid/globe placement unchanged). */
export const ENTITY_RAIL_SPAWN_XY = { x: 0, y: 0 } as const;

export const ENTITY_CONTROL_GOVERNANCE =
  "RT-local entity registry commands only — spawn and delete in this editing session. " +
  "Not SA import, corpus promotion, or operational engagement authority. " +
  "Move via existing grid/globe drag workflow (move_entity).";

export function entityRegistryCommandsAllowed(sessionState: string): boolean {
  return scenarioControlStates(sessionState).applyScenario;
}
