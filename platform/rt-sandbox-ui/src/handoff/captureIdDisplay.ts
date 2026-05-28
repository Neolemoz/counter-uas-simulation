export function shortCaptureId(id: string): string {
  return id.length > 12 ? `${id.slice(0, 8)}…` : id;
}
