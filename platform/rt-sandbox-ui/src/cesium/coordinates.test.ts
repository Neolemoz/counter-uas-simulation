import { describe, expect, it } from "vitest";
import { cartographicToWorld, worldToCartographic } from "./coordinates";
import { FICTIONAL_GEOREF_ANCHOR } from "./constants";

describe("worldToCartographic", () => {
  it("maps origin ENU to anchor height", () => {
    const c = worldToCartographic(0, 0, 0);
    const lon0 = (FICTIONAL_GEOREF_ANCHOR.lon_deg * Math.PI) / 180;
    const lat0 = (FICTIONAL_GEOREF_ANCHOR.lat_deg * Math.PI) / 180;
    expect(c.longitude).toBeCloseTo(lon0, 8);
    expect(c.latitude).toBeCloseTo(lat0, 8);
    expect(c.height).toBeCloseTo(FICTIONAL_GEOREF_ANCHOR.h_m, 4);
  });

  it("round-trips ENU within tolerance", () => {
    const x = 120;
    const y = -80;
    const z = 25;
    const c = worldToCartographic(x, y, z);
    const back = cartographicToWorld(c);
    expect(back.x).toBeCloseTo(x, 0);
    expect(back.y).toBeCloseTo(y, 0);
    expect(back.z).toBeCloseTo(z, 0);
  });

});
