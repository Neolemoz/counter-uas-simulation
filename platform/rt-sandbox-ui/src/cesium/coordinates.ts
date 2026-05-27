import { Cartesian3 } from "cesium";
import { FICTIONAL_GEOREF_ANCHOR } from "./constants";

const M_PER_DEG_LAT = 111_320;

export interface CartographicRad {
  longitude: number;
  latitude: number;
  height: number;
}

/** ENU meters (scenario-local) → radians cartographic at fictional anchor. */
export function worldToCartographic(
  x_m: number,
  y_m: number,
  z_m = 0,
): CartographicRad {
  const lon0 = (FICTIONAL_GEOREF_ANCHOR.lon_deg * Math.PI) / 180;
  const lat0 = (FICTIONAL_GEOREF_ANCHOR.lat_deg * Math.PI) / 180;
  const cosLat = Math.cos(lat0) || 1;
  const dLon = x_m / (M_PER_DEG_LAT * cosLat);
  const dLat = y_m / M_PER_DEG_LAT;
  return {
    longitude: lon0 + (dLon * Math.PI) / 180,
    latitude: lat0 + (dLat * Math.PI) / 180,
    height: FICTIONAL_GEOREF_ANCHOR.h_m + z_m,
  };
}

export function worldToCartesian(x_m: number, y_m: number, z_m = 0): Cartesian3 {
  const c = worldToCartographic(x_m, y_m, z_m);
  return Cartesian3.fromRadians(c.longitude, c.latitude, c.height);
}

/** Approximate ENU from cartographic (inverse of worldToCartographic). */
export function cartographicToWorld(c: CartographicRad): {
  x: number;
  y: number;
  z: number;
} {
  const lon0 = (FICTIONAL_GEOREF_ANCHOR.lon_deg * Math.PI) / 180;
  const lat0 = (FICTIONAL_GEOREF_ANCHOR.lat_deg * Math.PI) / 180;
  const cosLat = Math.cos(lat0) || 1;
  const dLonDeg = ((c.longitude - lon0) * 180) / Math.PI;
  const dLatDeg = ((c.latitude - lat0) * 180) / Math.PI;
  return {
    x: dLonDeg * M_PER_DEG_LAT * cosLat,
    y: dLatDeg * M_PER_DEG_LAT,
    z: c.height - FICTIONAL_GEOREF_ANCHOR.h_m,
  };
}
