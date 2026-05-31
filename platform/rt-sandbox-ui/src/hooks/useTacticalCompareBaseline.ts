import { useEffect, useRef, useState } from "react";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import { resolveTacticalRoleIds } from "@/cesium/tacticalTrajectoryLayer";

export function tacticalCompareSignature(
  state: TacticalStatePayload | null | undefined,
): string {
  if (!state) return "";
  const { targetId, interceptorId } = resolveTacticalRoleIds(state);
  const tti = state.tti_s;
  return `${interceptorId ?? ""}|${targetId ?? ""}|${tti ?? ""}`;
}

export function useTacticalCompareBaseline(
  sessionId: string | null,
  current: TacticalStatePayload | null | undefined,
): TacticalStatePayload | null {
  const priorRef = useRef<{
    sessionId: string;
    state: TacticalStatePayload;
    signature: string;
  } | null>(null);
  const [baseline, setBaseline] = useState<TacticalStatePayload | null>(null);

  useEffect(() => {
    if (!sessionId || !current) {
      priorRef.current = null;
      setBaseline(null);
      return;
    }

    const signature = tacticalCompareSignature(current);
    const prior = priorRef.current;

    if (!prior || prior.sessionId !== sessionId) {
      priorRef.current = { sessionId, state: current, signature };
      setBaseline(null);
      return;
    }

    if (prior.signature !== signature) {
      setBaseline(prior.state);
    } else {
      setBaseline(null);
    }

    priorRef.current = { sessionId, state: current, signature };
  }, [sessionId, current]);

  return baseline;
}
