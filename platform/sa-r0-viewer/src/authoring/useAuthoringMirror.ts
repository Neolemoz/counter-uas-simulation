import { useEffect, useState } from "react";
import { loadAuthoringManifest } from "./loadAuthoring";

/** True when a frozen authoring manifest mirror exists for the pack (PLAT-SA-A1). */
export function useAuthoringMirror(packId: string | null | undefined): boolean {
  const [hasMirror, setHasMirror] = useState(false);

  useEffect(() => {
    if (!packId) {
      setHasMirror(false);
      return;
    }
    loadAuthoringManifest(packId)
      .then((m) => setHasMirror(Boolean(m)))
      .catch(() => setHasMirror(false));
  }, [packId]);

  return hasMirror;
}
