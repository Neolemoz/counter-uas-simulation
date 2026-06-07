import type { MirrorEntity } from "@/cesium/entityMarkers";
import type { DefenseZoneConfig } from "@/cesium/defenseZoneConfig";
import { entitySvgCenter, metersToSvg } from "@/cesium/defenseZoneGeometry";
import type { PlanningCoverageCell } from "@/cesium/planningDrawing";
import type { EnuPoint } from "@/cesium/tacticalGeometry";
import type { RadarDomeConfig } from "@/cesium/sensorDomeLayer";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import {
  deriveRuntimeCoverageRenderModel,
  type RuntimeCoverageRenderModel,
} from "./runtimeCoverageRenderModel";

function cellRect(
  cell: PlanningCoverageCell,
  cellSize: number,
): { x: number; y: number; width: number; height: number } {
  const half = cell.sizeM / 2;
  const topLeft = entitySvgCenter(cell.center.x - half, cell.center.y + half, cellSize);
  const bottomRight = entitySvgCenter(cell.center.x + half, cell.center.y - half, cellSize);
  return {
    x: topLeft.cx,
    y: topLeft.cy,
    width: Math.max(1, bottomRight.cx - topLeft.cx),
    height: Math.max(1, bottomRight.cy - topLeft.cy),
  };
}

function corridorPath(points: EnuPoint[], cellSize: number): string {
  if (points.length < 2) return "";
  const [first, ...rest] = points;
  const start = entitySvgCenter(first.x, first.y, cellSize);
  const segments = rest.map((point) => {
    const { cx, cy } = entitySvgCenter(point.x, point.y, cellSize);
    return `L ${cx} ${cy}`;
  });
  return `M ${start.cx} ${start.cy} ${segments.join(" ")}`;
}

function CoverageCells({
  cells,
  fill,
  stroke,
  testId,
  cellSize,
}: {
  cells: PlanningCoverageCell[];
  fill: string;
  stroke: string;
  testId: string;
  cellSize: number;
}) {
  return (
    <g data-testid={testId}>
      {cells.map((cell, index) => {
        const rect = cellRect(cell, cellSize);
        return (
          <rect
            key={`${testId}-${index}`}
            x={rect.x}
            y={rect.y}
            width={rect.width}
            height={rect.height}
            fill={fill}
            stroke={stroke}
            strokeWidth={0.5}
            pointerEvents="none"
          />
        );
      })}
    </g>
  );
}

export function RuntimeCoverageSvgOverlay({
  entities,
  cellSize,
  protectedCenterEntityId = null,
  defenseZoneConfig,
  radarDomeConfig,
  tacticalState = null,
  visible = false,
}: {
  entities: MirrorEntity[];
  cellSize: number;
  protectedCenterEntityId?: string | null;
  defenseZoneConfig?: Partial<DefenseZoneConfig>;
  radarDomeConfig?: Partial<RadarDomeConfig>;
  tacticalState?: TacticalStatePayload | null;
  visible?: boolean;
}) {
  if (!visible) return null;

  const model = deriveRuntimeCoverageRenderModel({
    entities,
    protectedCenterEntityId,
    defenseZoneConfig,
    radarDomeConfig,
    tacticalState,
  });

  if (!model.ready) return null;

  return (
    <RuntimeCoverageSvgOverlayView model={model} cellSize={cellSize} />
  );
}

export function RuntimeCoverageSvgOverlayView({
  model,
  cellSize,
}: {
  model: RuntimeCoverageRenderModel;
  cellSize: number;
}) {
  return (
    <g data-testid="runtime-coverage-svg-overlay" pointerEvents="none">
      <CoverageCells
        cells={model.coveredCells}
        fill="rgba(34, 197, 94, 0.22)"
        stroke="rgba(34, 197, 94, 0.55)"
        testId="runtime-coverage-svg-covered"
        cellSize={cellSize}
      />
      <CoverageCells
        cells={model.uncoveredCells}
        fill="rgba(239, 68, 68, 0.18)"
        stroke="rgba(239, 68, 68, 0.5)"
        testId="runtime-coverage-svg-uncovered"
        cellSize={cellSize}
      />
      {model.blindSpotHints.map((cell, index) => {
        const { cx, cy } = entitySvgCenter(cell.center.x, cell.center.y, cellSize);
        return (
          <circle
            key={`blind-${index}`}
            cx={cx}
            cy={cy}
            r={Math.max(2, metersToSvg(cell.sizeM * 0.15, cellSize))}
            fill="rgba(248, 113, 113, 0.88)"
            stroke="rgba(127, 29, 29, 0.98)"
            strokeWidth={0.75}
            data-testid={`runtime-coverage-svg-blind-spot-${index}`}
          />
        );
      })}
      {model.majorUncoveredSectors.map((sector, index) => {
        const { cx, cy } = entitySvgCenter(sector.centroid.x, sector.centroid.y, cellSize);
        return (
          <g key={`sector-${index}`} data-testid={`runtime-coverage-svg-sector-${index}`}>
            <circle
              cx={cx}
              cy={cy}
              r={4}
              fill="rgba(245, 158, 11, 0.9)"
              stroke="rgba(120, 53, 15, 0.98)"
              strokeWidth={0.75}
            />
            <text
              x={cx}
              y={cy - 6}
              textAnchor="middle"
              fontSize={7}
              fontWeight={600}
              fill="rgba(254, 243, 199, 0.95)"
            >
              {sector.sector}
            </text>
          </g>
        );
      })}
      {model.coveredCorridorPolylines.map((points, index) => (
        <path
          key={`corridor-covered-${index}`}
          d={corridorPath(points, cellSize)}
          fill="none"
          stroke="rgba(34, 197, 94, 0.85)"
          strokeWidth={2}
          data-testid={`runtime-coverage-svg-corridor-covered-${index}`}
        />
      ))}
      {model.uncoveredCorridorPolylines.map((points, index) => (
        <path
          key={`corridor-uncovered-${index}`}
          d={corridorPath(points, cellSize)}
          fill="none"
          stroke="rgba(239, 68, 68, 0.9)"
          strokeWidth={2.5}
          data-testid={`runtime-coverage-svg-corridor-uncovered-${index}`}
        />
      ))}
    </g>
  );
}
