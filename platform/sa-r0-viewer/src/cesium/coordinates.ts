import type { ReplaySaBundle } from "@/replay/bundleSchema";

const M_PER_DEG_LAT = 111_320;

export function enuToCartographic(
  bundle: ReplaySaBundle,
  x_m: number,
  y_m: number,
  z_m = 0,
): { longitude: number; latitude: number; height: number } {
  const anchor = bundle.georef_display.anchor;
  const lon0 = (anchor.lon_deg * Math.PI) / 180;
  const lat0 = (anchor.lat_deg * Math.PI) / 180;
  const cosLat = Math.cos(lat0) || 1;
  const dLon = x_m / (M_PER_DEG_LAT * cosLat);
  const dLat = y_m / M_PER_DEG_LAT;
  const lon = lon0 + (dLon * Math.PI) / 180;
  const lat = lat0 + (dLat * Math.PI) / 180;
  return {
    longitude: lon,
    latitude: lat,
    height: anchor.h_m + z_m,
  };
}
