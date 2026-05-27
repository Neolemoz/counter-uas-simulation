import { generateContourPolylines, RT_RIDGE_TERRAIN } from "@/cesium/rtFictionalTerrain";
import { WORLD_BOUNDS } from "@/world/bounds";
import { worldToCell } from "@/world/gridCoords";

export function TerrainSvgOverlay({
  cellSize,
  showContours = false,
}: {
  cellSize: number;
  showContours?: boolean;
}) {
  const { x, y } = WORLD_BOUNDS;
  const contours = showContours ? generateContourPolylines().slice(0, 24) : [];

  return (
    <g data-testid="terrain-svg-overlay" pointerEvents="none" opacity={0.55}>
      {RT_RIDGE_TERRAIN.ridge_features.map((ridge) => {
        const pts = ridge.polyline_enu_m
          .map(([wx, wy]) => {
            const cell = worldToCell(wx, wy);
            return `${cell.col * cellSize + cellSize / 2},${cell.row * cellSize + cellSize / 2}`;
          })
          .join(" ");
        return (
          <polyline
            key={ridge.ridge_id}
            points={pts}
            fill="none"
            stroke="rgb(251 191 36)"
            strokeWidth={1.5}
            strokeDasharray="3 2"
          />
        );
      })}
      {contours.map((contour, i) => {
        const pts = contour.points_enu_m
          .map(([wx, wy]) => {
            const cell = worldToCell(wx, wy);
            return `${cell.col * cellSize + cellSize / 2},${cell.row * cellSize + cellSize / 2}`;
          })
          .join(" ");
        return (
          <polyline
            key={`contour-${contour.level_m}-${i}`}
            points={pts}
            fill="none"
            stroke="rgb(148 163 184)"
            strokeWidth={0.75}
            strokeDasharray="2 2"
            opacity={0.7}
          />
        );
      })}
      <text
        x={worldToCell(x.min + 40, y.max - 40).col * cellSize}
        y={worldToCell(x.min + 40, y.max - 40).row * cellSize}
        fill="rgb(148 163 184)"
        fontSize={8}
      >
        fictional terrain contour (explanatory)
      </text>
    </g>
  );
}
