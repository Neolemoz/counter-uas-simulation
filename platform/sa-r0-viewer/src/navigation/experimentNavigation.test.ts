import { describe, expect, it } from "vitest";
import { packIdFromBundlePath } from "./experimentNavigation";

describe("packIdFromBundlePath", () => {
  it("extracts pack id from demo_ridge_defense path", () => {
    expect(packIdFromBundlePath("fixtures/sa_r0/demo_ridge_defense")).toBe("ridge_defense");
  });

  it("extracts from index.json path", () => {
    expect(packIdFromBundlePath("fixtures/sa_r0/demo_valley_ingress/index.json")).toBe(
      "valley_ingress",
    );
  });

  it("returns null for unparseable paths", () => {
    expect(packIdFromBundlePath("unknown/path")).toBeNull();
  });
});
