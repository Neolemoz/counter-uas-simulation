import { navigateToCorpusEntryById, type NavigateHooks } from "./navigateToCorpusEntry";
import { readCorpusEntryFromUrl, useCorpusStore } from "./useCorpusStore";

export async function tryResolveCorpusEntryFromUrl(hooks: NavigateHooks): Promise<boolean> {
  const entryId = readCorpusEntryFromUrl();
  if (!entryId) return false;

  useCorpusStore.getState().setSelectedEntryId(entryId);
  useCorpusStore.getState().setBrowserOpen(true);
  await navigateToCorpusEntryById(entryId, hooks);
  return true;
}
