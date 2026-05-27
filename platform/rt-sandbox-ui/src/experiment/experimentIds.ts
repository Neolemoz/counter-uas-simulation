export function shortId(id: string | null | undefined): string | null {
  if (!id) return null;
  return id.length > 8 ? id.slice(0, 8) : id;
}
