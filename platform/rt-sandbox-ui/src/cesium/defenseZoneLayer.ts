import {
  Cartesian2,
  Cartesian3,
  Color,
  Entity,
  LabelStyle,
  PolygonHierarchy,
  VerticalOrigin,
  Viewer,
} from "cesium";
import type { MirrorEntity } from "./entityMarkers";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import {
  sampleTerrainHeight,
} from "./rtFictionalTerrain";
import {
  DEFAULT_DEFENSE_ZONE_CONFIG,
  isProtectedAsset,
  normalizedDefenseZoneConfig,
  type DefenseZoneConfig,
  type DefenseZoneRenderOptions,
  type DefenseZoneSizes,
} from "./defenseZoneConfig";
import {
  labelAnchorWorld,
  labelAzimuthDegForDefenseZone,
  defenseLabelPixelOffset,
  zoneBoundaryPositionsGrounded,
} from "./defenseZoneGeometry";
import { cameraHeightM } from "./cameraHelpers";
import {
  groundedSurfaceZ,
  markerDisplayZ,
  ZONE_SURFACE_LIFT_M,
  zoneLabelLiftM,
} from "./terrainGrounding";
import {
  PROTECTED_CENTER_ZONE_LABEL,
  resolveDefenseZoneEntityVisual,
} from "./defenseZoneVisualState";

const DEFENSE_PREFIX = "rt-defense-zone-";

/** Protected core — red. */
const DEFENSE_CORE_RGB = "248, 113, 113";
const DEFENSE_CORE_EDGE = "252, 165, 165";
/** Engagement — yellow. */
const DEFENSE_ENGAGE_RGB = "250, 204, 21";
const DEFENSE_ENGAGE_EDGE = "253, 224, 71";
/** Outer warning — blue. */
const DEFENSE_WARNING_RGB = "96, 165, 250";
const DEFENSE_WARNING_EDGE = "147, 197, 253";

export type DefenseZoneLevelSuffix = "core" | "mid" | "warning";

interface DefenseZoneLevelSpec {
  suffix: DefenseZoneLevelSuffix;
  sizeKey: keyof DefenseZoneSizes;
  width: number;
  alpha: number;
  fillRgb: string;
  edgeRgb: string;
  label: string;
  halo?: boolean;
}

/** Core strongest → mid medium → warning softest (pass #2). */
export const DEFENSE_ZONE_LEVELS: DefenseZoneLevelSpec[] = [
  {
    suffix: "core",
    sizeKey: "coreM",
    width: 3.5,
    alpha: 0.96,
    fillRgb: DEFENSE_CORE_RGB,
    edgeRgb: DEFENSE_CORE_EDGE,
    label: "Core",
    halo: true,
  },
  {
    suffix: "mid",
    sizeKey: "engageM",
    width: 2.5,
    alpha: 0.68,
    fillRgb: DEFENSE_ENGAGE_RGB,
    edgeRgb: DEFENSE_ENGAGE_EDGE,
    label: "Engagement",
  },
  {
    suffix: "warning",
    sizeKey: "warningM",
    width: 1.75,
    alpha: 0.42,
    fillRgb: DEFENSE_WARNING_RGB,
    edgeRgb: DEFENSE_WARNING_EDGE,
    label: "Warning",
  },
];

function addRangeLabel(
  viewer: Viewer,
  id: string,
  position: Cartesian3,
  text: string,
  edgeRgb: string,
  fade: number,
  emphasis: number,
  suffix: DefenseZoneLevelSuffix,
  cameraHeightMeters: number,
): void {
  const labelAlpha = Math.min(1, 0.98 * fade * emphasis);
  const pixel = defenseLabelPixelOffset(suffix);
  const fontSize =
    cameraHeightMeters > 3800 ? 10 : cameraHeightMeters > 2200 ? 11 : 12;
  viewer.entities.add(
    new Entity({
      id,
      position,
      label: {
        text,
        font: `bold ${fontSize}px sans-serif`,
        fillColor: Color.fromCssColorString(`rgba(${edgeRgb}, ${labelAlpha})`),
        outlineColor: Color.fromCssColorString("rgba(2, 6, 23, 0.98)"),
        outlineWidth: 3,
        style: LabelStyle.FILL_AND_OUTLINE,
        verticalOrigin: VerticalOrigin.BOTTOM,
        pixelOffset: new Cartesian2(pixel.x, pixel.y),
        showBackground: true,
        backgroundColor: Color.fromCssColorString("rgba(2, 6, 23, 0.92)"),
        backgroundPadding: new Cartesian2(7, 5),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    }),
  );
}

function addBoundary(
  viewer: Viewer,
  id: string,
  positions: Cartesian3[],
  edgeRgb: string,
  ringAlpha: number,
  ringWidth: number,
  halo?: boolean,
): void {
  const edgeColor = Color.fromCssColorString(`rgba(${edgeRgb}, ${Math.min(1, ringAlpha)})`);
  if (halo && ringAlpha > 0.18) {
    viewer.entities.add(
      new Entity({
        id: `${id}-halo`,
        polyline: {
          positions,
          width: ringWidth + 2.5,
          material: Color.fromCssColorString(`rgba(${edgeRgb}, ${ringAlpha * 0.2})`),
        },
      }),
    );
  }
  viewer.entities.add(
    new Entity({
      id,
      polyline: { positions, width: ringWidth, material: edgeColor },
    }),
  );
}

function removeDefenseEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(DEFENSE_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

function entityGroundPose(
  ent: MirrorEntity,
  applyTerrainDisplay: boolean,
): { x: number; y: number; z: number; groundZ: number } {
  const x = Number(ent.pose.x ?? 0);
  const y = Number(ent.pose.y ?? 0);
  const zReg = Number(ent.pose.z ?? 10);
  const z = applyTerrainDisplay
    ? markerDisplayZ(x, y, zReg, true)
    : zReg + sampleTerrainHeight(x, y);
  const groundZ = groundedSurfaceZ(x, y, ZONE_SURFACE_LIFT_M);
  return { x, y, z, groundZ };
}

function renderDefenseZone(
  viewer: Viewer,
  ent: MirrorEntity,
  config: DefenseZoneConfig,
  applyTerrainDisplay: boolean,
  options: DefenseZoneRenderOptions,
  visual: ReturnType<typeof resolveDefenseZoneEntityVisual>,
): void {
  const { x, y } = entityGroundPose(ent, applyTerrainDisplay);
  const cameraHeight = cameraHeightM(viewer);
  const { fade, emphasis, isSelected, isDesignated, showZoneLabels } = visual;
  const showLabels =
    options.showLabels !== false && config.showLabels !== false && showZoneLabels;
  const warningSize = config.sizes.warningM;
  const coreSize = config.sizes.coreM;

  if (config.shape === "circle") {
    const warningRing = zoneBoundaryPositionsGrounded(
      "circle",
      x,
      y,
      warningSize,
      ZONE_SURFACE_LIFT_M,
    ).slice(0, -1);
    viewer.entities.add(
      new Entity({
        id: `${DEFENSE_PREFIX}fill-${ent.entity_id}`,
        polygon: {
          hierarchy: new PolygonHierarchy(warningRing),
          perPositionHeight: true,
          material: Color.fromCssColorString(
            `rgba(${DEFENSE_WARNING_RGB}, ${(isSelected ? 0.028 : 0.016) * fade * emphasis})`,
          ),
          outline: false,
        },
      }),
    );
    const coreRing = zoneBoundaryPositionsGrounded(
      "circle",
      x,
      y,
      coreSize,
      ZONE_SURFACE_LIFT_M + 0.05,
    ).slice(0, -1);
    viewer.entities.add(
      new Entity({
        id: `${DEFENSE_PREFIX}core-fill-${ent.entity_id}`,
        polygon: {
          hierarchy: new PolygonHierarchy(coreRing),
          perPositionHeight: true,
          material: Color.fromCssColorString(
            `rgba(${DEFENSE_CORE_RGB}, ${(isSelected ? 0.14 : 0.09) * fade * emphasis})`,
          ),
          outline: false,
        },
      }),
    );
  } else {
    const corners = zoneBoundaryPositionsGrounded(
      "rectangle",
      x,
      y,
      warningSize,
      ZONE_SURFACE_LIFT_M,
    ).slice(0, 4);
    viewer.entities.add(
      new Entity({
        id: `${DEFENSE_PREFIX}fill-${ent.entity_id}`,
        polygon: {
          hierarchy: new PolygonHierarchy(corners),
          perPositionHeight: true,
          material: Color.fromCssColorString(
            `rgba(${DEFENSE_WARNING_RGB}, ${(isSelected ? 0.024 : 0.014) * fade * emphasis})`,
          ),
          outline: false,
        },
      }),
    );
    const coreCorners = zoneBoundaryPositionsGrounded(
      "rectangle",
      x,
      y,
      coreSize,
      ZONE_SURFACE_LIFT_M + 0.15,
    ).slice(0, 4);
    viewer.entities.add(
      new Entity({
        id: `${DEFENSE_PREFIX}core-fill-${ent.entity_id}`,
        polygon: {
          hierarchy: new PolygonHierarchy(coreCorners),
          perPositionHeight: true,
          material: Color.fromCssColorString(
            `rgba(${DEFENSE_CORE_RGB}, ${(isSelected ? 0.12 : 0.08) * fade * emphasis})`,
          ),
          outline: false,
        },
      }),
    );
  }

  const renderOrder: DefenseZoneLevelSuffix[] = ["warning", "mid", "core"];
  for (const suffix of renderOrder) {
    const level = DEFENSE_ZONE_LEVELS.find((l) => l.suffix === suffix);
    if (!level) continue;
    const sizeM = config.sizes[level.sizeKey];
    const ringAlpha = level.alpha * fade * emphasis;
    const ringWidth = level.width * (isSelected ? 1.35 : 1);
    const positions = zoneBoundaryPositionsGrounded(
      config.shape,
      x,
      y,
      sizeM,
      ZONE_SURFACE_LIFT_M,
    );

    addBoundary(
      viewer,
      `${DEFENSE_PREFIX}${level.suffix}-${ent.entity_id}`,
      positions,
      level.edgeRgb,
      ringAlpha,
      ringWidth,
      level.halo,
    );

    if (showLabels) {
      const anchor = labelAnchorWorld(
        config.shape,
        x,
        y,
        sizeM,
        labelAzimuthDegForDefenseZone(level.suffix),
      );
      addRangeLabel(
        viewer,
        `${DEFENSE_PREFIX}${level.suffix}-label-${ent.entity_id}`,
        worldToCartesian(
          anchor.wx,
          anchor.wy,
          groundedSurfaceZ(anchor.wx, anchor.wy, ZONE_SURFACE_LIFT_M) +
            zoneLabelLiftM(level.suffix),
        ),
        level.label,
        level.edgeRgb,
        fade,
        emphasis,
        level.suffix,
        cameraHeight,
      );
    }
  }

  if (isDesignated && showLabels && config.shape === "circle") {
    addRangeLabel(
      viewer,
      `${DEFENSE_PREFIX}protected-center-label-${ent.entity_id}`,
      worldToCartesian(
        x,
        y,
        groundedSurfaceZ(x, y, ZONE_SURFACE_LIFT_M) + zoneLabelLiftM("core") + 8,
      ),
      PROTECTED_CENTER_ZONE_LABEL,
      "52, 211, 153",
      fade,
      emphasis,
      "core",
      cameraHeight,
    );
  }
}

export function syncDefenseZoneLayer(
  viewer: Viewer | null | undefined,
  entities: MirrorEntity[],
  show: boolean,
  applyTerrainDisplay: boolean,
  options: DefenseZoneRenderOptions = {},
): void {
  if (!isViewerUsable(viewer)) return;
  removeDefenseEntities(viewer);
  if (!show) return;

  const config = normalizedDefenseZoneConfig(options.config ?? DEFAULT_DEFENSE_ZONE_CONFIG);
  const selected = entities.find((ent) => ent.entity_id === options.selectedEntityId) ?? null;
  const selectedDefenseOnly =
    options.selectedOnly === true && isProtectedAsset(selected?.entity_type ?? "");
  const showAllDefenseZones = !selectedDefenseOnly;
  const selectedDefensePresent = isProtectedAsset(selected?.entity_type ?? "");

  const labelsEnabled = options.showLabels !== false && config.showLabels !== false;

  for (const ent of entities) {
    if (!ent.entity_id || !isProtectedAsset(ent.entity_type)) continue;
    if (selectedDefenseOnly && ent.entity_id !== selected?.entity_id) continue;

    const visual = resolveDefenseZoneEntityVisual({
      entityId: ent.entity_id,
      selectedEntityId: options.selectedEntityId,
      protectedCenterEntityId: options.protectedCenterEntityId,
      selectedDefensePresent,
      showAllDefenseZones,
      labelsEnabled,
    });
    renderDefenseZone(viewer, ent, config, applyTerrainDisplay, options, visual);
  }
}

export function clearDefenseZoneLayer(viewer: Viewer | null | undefined): void {
  if (!isViewerUsable(viewer)) return;
  removeDefenseEntities(viewer);
}
