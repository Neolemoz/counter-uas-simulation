import { useCallback, useEffect, useState } from "react";
import { checkLiveRuntimePreflight } from "@/bridge/preflightCommands";
import type { LivePreflightResult } from "@/runtime/livePreflight";
import type { SessionRuntimeProfile } from "@/runtime/sessionRuntimeProfile";

export function useLiveRuntimePreflight(profile: SessionRuntimeProfile) {
  const [livePreflight, setLivePreflight] = useState<LivePreflightResult | null>(
    null,
  );
  const [preflightLoading, setPreflightLoading] = useState(false);
  const [preflightError, setPreflightError] = useState<string | null>(null);

  const refreshPreflight = useCallback(async () => {
    setPreflightLoading(true);
    try {
      const resp = await checkLiveRuntimePreflight();
      if (!resp.ok) {
        setPreflightError(resp.message ?? resp.error_code ?? "preflight failed");
        setLivePreflight(resp.preflight ?? null);
        return;
      }
      setPreflightError(null);
      setLivePreflight(resp.preflight ?? null);
    } catch (err) {
      setPreflightError(err instanceof Error ? err.message : "preflight error");
    } finally {
      setPreflightLoading(false);
    }
  }, []);

  useEffect(() => {
    if (profile !== "live") {
      setLivePreflight(null);
      setPreflightError(null);
      return;
    }
    void refreshPreflight();
  }, [profile, refreshPreflight]);

  return {
    livePreflight,
    preflightLoading,
    preflightError,
    refreshPreflight,
  };
}
