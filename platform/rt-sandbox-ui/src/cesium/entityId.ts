/** Cesium entity id prefix for RT mirror markers (PLAT-RT-T5). */

export const RT_ENTITY_PREFIX = "rt-entity-";

export function toCesiumEntityId(entityId: string): string {
  return `${RT_ENTITY_PREFIX}${entityId}`;
}

export function parseRtEntityId(cesiumEntityId: string | undefined): string | null {
  if (!cesiumEntityId || !cesiumEntityId.startsWith(RT_ENTITY_PREFIX)) {
    return null;
  }
  const id = cesiumEntityId.slice(RT_ENTITY_PREFIX.length);
  return id.length > 0 ? id : null;
}
