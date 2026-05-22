import { useEffect, useState } from "react";
import { loadFederationPublicationCollection } from "./loadFederationArtifacts";
import type { ReplayFederationPublicationCollection } from "./federationSchema";
import { useFederationStore } from "./useFederationStore";

export function FederationPublicationCollectionPanel() {
  const [collection, setCollection] = useState<ReplayFederationPublicationCollection | null>(
    null,
  );
  const corpusGroupId = useFederationStore((s) => s.corpusGroupId);

  useEffect(() => {
    loadFederationPublicationCollection()
      .then(setCollection)
      .catch(() => setCollection(null));
  }, []);

  if (!collection?.members?.length) {
    return <p className="text-xs text-slate-500">No publication collection members.</p>;
  }

  const members = corpusGroupId
    ? collection.members.filter((m) => m.corpus_group_id === corpusGroupId)
    : collection.members;

  return (
    <div className="space-y-2 text-xs">
      <p className="font-mono text-[10px] text-slate-500">{collection.collection_id}</p>
      <ul className="space-y-0.5 font-mono text-[10px] text-slate-500">
        {members.map((m) => (
          <li key={`${m.corpus_group_id}-${m.artifact_path}`}>
            <span className="text-slate-400">{m.artifact_kind}</span> · {m.artifact_path}
          </li>
        ))}
      </ul>
    </div>
  );
}
