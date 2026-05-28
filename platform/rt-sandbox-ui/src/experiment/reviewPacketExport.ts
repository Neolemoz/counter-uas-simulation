import { exportReviewPacketJson } from "./reviewPacketPreview";
import type { ExperimentReviewPacket } from "./reviewPacketSchema";

export { exportReviewPacketJson };

export function suggestedReviewPacketFilename(packet: ExperimentReviewPacket): string {
  const safe = packet.packet_id.replace(/[^\w.-]+/g, "_");
  return `${safe}.json`;
}

export function downloadReviewPacket(packet: ExperimentReviewPacket): void {
  const json = exportReviewPacketJson(packet);
  const blob = new Blob([json], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = suggestedReviewPacketFilename(packet);
  a.click();
  URL.revokeObjectURL(url);
}

export async function copyReviewPacketJson(
  packet: ExperimentReviewPacket,
): Promise<{ ok: true } | { ok: false; error: string }> {
  const json = exportReviewPacketJson(packet);
  if (typeof navigator === "undefined" || !navigator.clipboard?.writeText) {
    return { ok: false, error: "clipboard unavailable" };
  }
  try {
    await navigator.clipboard.writeText(json);
    return { ok: true };
  } catch (err) {
    return {
      ok: false,
      error: err instanceof Error ? err.message : String(err),
    };
  }
}
