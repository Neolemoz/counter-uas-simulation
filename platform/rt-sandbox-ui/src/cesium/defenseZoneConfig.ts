/** UI-only protected-asset defense zone configuration (Gazebo-aligned geometry). */

export type DefenseZoneShape = "circle" | "rectangle";

export interface DefenseZoneSizes {
  /** Protected core radius or half-extent (m). */
  coreM: number;
  /** Engagement zone radius or half-extent (m). */
  engageM: number;
  /** Outer warning zone radius or half-extent (m). */
  warningM: number;
}

export interface DefenseZoneConfig {
  shape: DefenseZoneShape;
  sizes: DefenseZoneSizes;
  showLabels: boolean;
}

export interface DefenseZoneRenderOptions {
  show?: boolean;
  selectedEntityId?: string | null;
  /** Explicit session protected center — never inferred from selection. */
  protectedCenterEntityId?: string | null;
  selectedOnly?: boolean;
  showLabels?: boolean;
  config?: DefenseZoneConfig;
}

export const DEFAULT_DEFENSE_ZONE_SIZES: DefenseZoneSizes = {
  coreM: 50,
  engageM: 150,
  warningM: 280,
};

export const DEFAULT_DEFENSE_ZONE_CONFIG: DefenseZoneConfig = {
  shape: "circle",
  sizes: DEFAULT_DEFENSE_ZONE_SIZES,
  showLabels: true,
};

export function isProtectedAsset(entityType: string): boolean {
  return entityType === "waypoint_marker";
}

export function normalizedDefenseZoneSizes(
  sizes?: Partial<DefenseZoneSizes>,
): DefenseZoneSizes {
  const coreM = Math.max(10, Number(sizes?.coreM ?? DEFAULT_DEFENSE_ZONE_SIZES.coreM));
  const engageM = Math.max(
    coreM + 10,
    Number(sizes?.engageM ?? DEFAULT_DEFENSE_ZONE_SIZES.engageM),
  );
  const warningM = Math.max(
    engageM + 10,
    Number(sizes?.warningM ?? DEFAULT_DEFENSE_ZONE_SIZES.warningM),
  );
  return { coreM, engageM, warningM };
}

export function normalizedDefenseZoneConfig(
  config?: Partial<DefenseZoneConfig>,
): DefenseZoneConfig {
  const shape = config?.shape === "rectangle" ? "rectangle" : "circle";
  return {
    shape,
    sizes: normalizedDefenseZoneSizes(config?.sizes),
    showLabels: config?.showLabels !== false,
  };
}
