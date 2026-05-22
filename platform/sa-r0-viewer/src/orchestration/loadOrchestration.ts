import {
  experimentRunQueueSchema,
  orchestrationIndexSchema,
  validationMirrorSchema,
  type ExperimentRunQueue,
  type ValidationMirror,
} from "./orchestrationSchema";

const INDEX_URL = "/demo/orchestration/index.json";

export async function loadOrchestrationIndex(): Promise<
  { id: string; kind: string; url: string }[]
> {
  const res = await fetch(INDEX_URL);
  if (!res.ok) return [];
  const data = orchestrationIndexSchema.parse(await res.json());
  return data.entries;
}

export async function loadOrchestrationQueue(queueId: string): Promise<ExperimentRunQueue | null> {
  const url = `/demo/orchestration/queues/${queueId}.json`;
  const res = await fetch(url);
  if (!res.ok) return null;
  return experimentRunQueueSchema.parse(await res.json());
}

export async function loadValidationMirror(packId: string): Promise<ValidationMirror | null> {
  const url = `/demo/orchestration/validation_mirrors/${packId}_validation_mirror.json`;
  const res = await fetch(url);
  if (!res.ok) return null;
  return validationMirrorSchema.parse(await res.json());
}

export function readOrchestrationQueueFromUrl(): string | null {
  if (typeof window === "undefined") return null;
  return new URLSearchParams(window.location.search).get("orchestration_queue");
}
