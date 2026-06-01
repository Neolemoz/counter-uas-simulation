import { DEFENSE_ZONE_LEVELS, type DefenseZoneLevelSuffix } from "@/cesium/defenseZoneLayer";
import {
  DEFAULT_DEFENSE_ZONE_CONFIG,
  isProtectedAsset,
  normalizedDefenseZoneConfig,
  type DefenseZoneConfig,
} from "@/cesium/defenseZoneConfig";
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
  config = DEFAULT_DEFENSE_ZONE_CONFIG,
  visible = true,
}: {
  entities: UiEntity[];
  cellSize: number;
  selectedEntityId?: string | null;
  config?: DefenseZoneConfig;
  visible?: boolean;
}) {
  if (!visible) return null;

  const zoneConfig = normalizedDefenseZoneConfig(config);
  const protectedEntities = entities.filter((ent) => isProtectedAsset(ent.entity_type));

  return (
    <g data-testid="defense-zone-svg-overlay" pointerEvents="none">
      {protectedEntities.map((ent) => {
        const x = Number(ent.pose.x ?? 0);
        const y = Number(ent.pose.y ?? 0);
        const { cx, cy } = entitySvgCenter(x, y, cellSize);
        const selected = ent.entity_id === selectedEntityId;
        const fade = selected ? 1 : 0.55;
        const showLabels = zoneConfig.showLabels && selected;
        const corePx = metersToSvg(zoneConfig.sizes.coreM, cellSize);
        const warningPx = metersToSvg(zoneConfig.sizes.warningM, cellSize);

        return (
          <g key={`defense-${ent.entity_id}`} opacity={fade}>
            {zoneConfig.shape === "circle" ? (
              <>
                <circle
                  cx={cx}
                  cy={cy}
                  r={warningPx}
                  fill={`rgba(${DEFENSE_ZONE_LEVELS[2].fillRgb}, 0.08)`}
                />
                <circle
                  cx={cx}
                  cy={cy}
                  r={corePx}
                  fill={`rgba(${DEFENSE_ZONE_LEVELS[0].fillRgb}, 0.16)`}
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
          </g>
        );
      })}
    </g>
  );
}
