import type { CompareSide } from "./experimentCompare";

export function TerrainCompareNote({ sideA, sideB }: { sideA: CompareSide; sideB: CompareSide }) {
  return (
    <p className="text-[10px] text-slate-500">
      Terrain context uses the shared fictional RT heightmap (PLAT-RT-V2). A:{" "}
      {sideA.terrainNote ?? "—"} · B:{" "}
      {sideB.terrainNote ?? "—"} — not sensor or terrain truth.
    </p>
  );
}
