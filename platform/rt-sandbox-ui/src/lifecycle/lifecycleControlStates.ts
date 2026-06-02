/** UI enablement for lifecycle control bar (mirrors rt_bridge_contract_v1 session commands). */

export type LifecycleControlStates = {
  pause: boolean;
  resume: boolean;
  reset: boolean;
  stopSession: boolean;
};

export function lifecycleControlStates(sessionState: string): LifecycleControlStates {
  const running = sessionState === "running";
  const paused = sessionState === "paused";
  return {
    pause: running,
    resume: paused,
    reset: running || paused,
    stopSession: running || paused,
  };
}
