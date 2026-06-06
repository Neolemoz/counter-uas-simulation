import { DEFENSE_ZONE_LEVELS, type DefenseZoneLevelSuffix } from "@/cesium/defenseZoneLayer";
import {
  DEFAULT_DEFENSE_ZONE_CONFIG,
  isProtectedAsset,
  normalizedDefenseZoneConfig,
  type DefenseZoneConfig,
} from "@/cesium/defenseZoneConfig";
import {
  PROTECTED_CENTER_ZONE_LABEL,
  resolveDefenseZoneEntityVisual,
} from "@/cesium/defenseZoneVisualState";
import {
  entitySvgCenter,
  labelAzimuthDegForDefenseZone,
  defenseLabelPixelOffset,
  labelSvgAnchor,
  metersToSvg,
} from "@/cesium/defenseZoneGeometry";
import type { UiEntity } from "@/editing/localEntityMirror";

const RENDER_ORDER: DefenseZoneLevelSuffix[] = ["warning", "mid", "core"];

function SvgZoneLabel({
  x,
  y,
  text,
  stroke,
}: {
  x: number;
  y: number;
  text: string;
  stroke: string;
}) {
  const padX = 4;
  const padY = 2;
  const width = text.length * 5.2 + padX * 2;
  const height = 11 + padY * 2;
  return (
    <g>
      <rect
        x={x - width / 2}
        y={y - height + 2}
        width={width}
        height={height}
        rx={2}
        fill="rgba(2, 6, 23, 0.92)"
        stroke={stroke}
        strokeWidth={0.5}
        strokeOpacity={0.65}
      />
      <text
        x={x}
        y={y}
        textAnchor="middle"
        fontSize={8}
        fontWeight={600}
        fontFamily="sans-serif"
        fill={stroke}
      >
        {text}
      </text>
    </g>
  );
}

export function DefenseZoneSvgOverlay({
  entities,
  cellSize,
  selectedEntityId,
  protectedCenterEntityId = null,
  config = DEFAULT_DEFENSE_ZONE_CONFIG,
  visible = true,
}: {
  entities: UiEntity[];
  cellSize: number;
  selectedEntityId?: string | null;
  protectedCenterEntityId?: string | null;
  config?: DefenseZoneConfig;
  visible?: boolean;
}) {
  if (!visible) return null;

  const zoneConfig = normalizedDefenseZoneConfig(config);
  const protectedEntities = entities.filter((ent) => isProtectedAsset(ent.entity_type));
  const selectedDefensePresent = protectedEntities.some(
    (ent) => ent.entity_id === selectedEntityId,
  );

  return (
    <g data-testid="defense-zone-svg-overlay" pointerEvents="none">
      {protectedEntities.map((ent) => {
        const x = Number(ent.pose.x ?? 0);
        const y = Number(ent.pose.y ?? 0);
        const { cx, cy } = entitySvgCenter(x, y, cellSize);
        const visual = resolveDefenseZoneEntityVisual({
          entityId: ent.entity_id,
          selectedEntityId,
          protectedCenterEntityId,
          selectedDefensePresent,
          showAllDefenseZones: true,
          labelsEnabled: zoneConfig.showLabels,
        });
        const { fade, emphasis, isDesignated, showZoneLabels } = visual;
        const showLabels = showZoneLabels;
        const corePx = metersToSvg(zoneConfig.sizes.coreM, cellSize);
        const warningPx = metersToSvg(zoneConfig.sizes.warningM, cellSize);
        const fillScale = emphasis;

        return (
          <g
            key={`defense-${ent.entity_id}`}
            opacity={fade}
            data-designated-protected-center={isDesignated ? "true" : undefined}
            data-defense-zone-emphasis={isDesignated ? "designated" : "candidate"}
          >
            {zoneConfig.shape === "circle" ? (
              <>
                <circle
                  cx={cx}
                  cy={cy}
                  r={warningPx}
                  fill={`rgba(${DEFENSE_ZONE_LEVELS[2].fillRgb}, ${(0.08 * fillScale).toFixed(3)})`}
                />
                <circle
                  cx={cx}
                  cy={cy}
                  r={corePx}
                  fill={`rgba(${DEFENSE_ZONE_LEVELS[0].fillRgb}, ${(isDesignated ? 0.22 : 0.16) * fillScale})`}
                  stroke={isDesignated ? "rgb(52, 211, 153)" : undefined}
                  strokeWidth={isDesignated ? 1.2 : 0}
                />
              </>
            ) : (
              <>
                <rect
                  x={cx - warningPx}
                  y={cy - warningPx}
                  width={warningPx * 2}
                  height={warningPx * 2}
                  fill={`rgba(${DEFENSE_ZONE_LEVELS[2].fillRgb}, 0.07)`}
                />
                <rect
                  x={cx - corePx}
                  y={cy - corePx}
                  width={corePx * 2}
                  height={corePx * 2}
                  fill={`rgba(${DEFENSE_ZONE_LEVELS[0].fillRgb}, 0.14)`}
                />
              </>
            )}
            {RENDER_ORDER.map((suffix) => {
              const level = DEFENSE_ZONE_LEVELS.find((l) => l.suffix === suffix);
              if (!level) return null;
              const sizePx = metersToSvg(zoneConfig.sizes[level.sizeKey], cellSize);
              const stroke = `rgb(${level.edgeRgb})`;

              const boundary =
                zoneConfig.shape === "rectangle" ? (
                  <rect
                    x={cx - sizePx}
                    y={cy - sizePx}
                    width={sizePx * 2}
                    height={sizePx * 2}
                    fill="none"
                    stroke={stroke}
                    strokeWidth={level.width * 0.65}
                    strokeOpacity={level.alpha}
                  />
                ) : (
                  <circle
                    cx={cx}
                    cy={cy}
                    r={sizePx}
                    fill="none"
                    stroke={stroke}
                    strokeWidth={level.width * 0.65}
                    strokeOpacity={level.alpha}
                  />
                );

              const labelAnchor = showLabels
                ? (() => {
                    const base = labelSvgAnchor(
                      zoneConfig.shape,
                      cx,
                      cy,
                      sizePx,
                      labelAzimuthDegForDefenseZone(suffix),
                    );
                    const nudge = defenseLabelPixelOffset(suffix);
                    return {
                      lx: base.lx + nudge.x * 0.35,
                      ly: base.ly + nudge.y * 0.35,
                    };
                  })()
                : null;

              return (
                <g key={`${ent.entity_id}-${suffix}`}>
                  {boundary}
                  {labelAnchor && (
                    <SvgZoneLabel
                      x={labelAnchor.lx}
                      y={labelAnchor.ly}
                      text={level.label}
                      stroke={stroke}
                    />
                  )}
                </g>
              );
            })}
            {isDesignated && showLabels && (
              <SvgZoneLabel
                x={cx}
                y={cy - corePx - 6}
                text={PROTECTED_CENTER_ZONE_LABEL}
                stroke="rgb(52, 211, 153)"
              />
            )}
          </g>
        );
      })}
    </g>
  );
}
