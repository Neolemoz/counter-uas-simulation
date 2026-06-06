/** Shared defense-zone emphasis when a protected center is explicitly designated. */

export const PROTECTED_CENTER_ZONE_LABEL = "Protected center";

export interface DefenseZoneEntityVisual {
  fade: number;
  emphasis: number;
  showZoneLabels: boolean;
  isDesignated: boolean;
  isSelected: boolean;
}

function legacyDefenseZoneFade(
  isSelected: boolean,
  selectedDefensePresent: boolean,
  showAllDefenseZones: boolean,
): number {
  if (isSelected) return 1;
  if (!showAllDefenseZones) return 1;
  if (selectedDefensePresent) return 0.34;
  return 0.62;
}

export function isDesignatedProtectedCenter(
  entityId: string,
  protectedCenterEntityId?: string | null,
): boolean {
  return (
    protectedCenterEntityId != null &&
    protectedCenterEntityId.length > 0 &&
    entityId === protectedCenterEntityId
  );
}

export function resolveDefenseZoneEntityVisual(input: {
  entityId: string;
  selectedEntityId?: string | null;
  protectedCenterEntityId?: string | null;
  selectedDefensePresent: boolean;
  showAllDefenseZones: boolean;
  labelsEnabled: boolean;
}): DefenseZoneEntityVisual {
  const isSelected = input.entityId === input.selectedEntityId;
  const isDesignated = isDesignatedProtectedCenter(
    input.entityId,
    input.protectedCenterEntityId,
  );
  const hasDesignatedCenter = input.protectedCenterEntityId != null;

  if (hasDesignatedCenter) {
    if (isDesignated) {
      return {
        fade: 1,
        emphasis: isSelected ? 1.52 : 1.4,
        showZoneLabels: input.labelsEnabled,
        isDesignated: true,
        isSelected,
      };
    }
    return {
      fade: isSelected ? 0.72 : 0.48,
      emphasis: isSelected ? 1.14 : 1,
      showZoneLabels: input.labelsEnabled && isSelected,
      isDesignated: false,
      isSelected,
    };
  }

  const fade = legacyDefenseZoneFade(
    isSelected,
    input.selectedDefensePresent,
    input.showAllDefenseZones,
  );
  return {
    fade,
    emphasis: isSelected ? 1.28 : 1,
    showZoneLabels: input.labelsEnabled && isSelected,
    isDesignated: false,
    isSelected,
  };
}
