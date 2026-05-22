import { useEffect, useState } from "react";
import { loadReplayCorpusIndex } from "../synthesis/loadSynthesis";
import { siblingsOf } from "./corpusNavigation";
import { navigateToCorpusEntryById, type NavigateHooks } from "./navigateToCorpusEntry";

type Props = {
  entryId: string | null;
  hooks: NavigateHooks;
};

export function CorpusSiblingsStrip({ entryId, hooks }: Props) {
  const [siblings, setSiblings] = useState<string[]>([]);

  useEffect(() => {
    if (!entryId) {
      setSiblings([]);
      return;
    }
    loadReplayCorpusIndex()
      .then((index) => setSiblings(siblingsOf(entryId, index)))
      .catch(() => setSiblings([]));
  }, [entryId]);

  if (!entryId || siblings.length === 0) return null;

  return (
    <p className="text-[10px] text-slate-500">
      Corpus siblings:{" "}
      {siblings.slice(0, 5).map((id, i) => (
        <span key={id}>
          {i > 0 ? ", " : ""}
          <button
            type="button"
            className="text-cyan-300 hover:underline"
            onClick={() => void navigateToCorpusEntryById(id, hooks)}
          >
            {id.split("__").pop()}
          </button>
        </span>
      ))}
      {siblings.length > 5 ? ` (+${siblings.length - 5} more)` : ""}
    </p>
  );
}
