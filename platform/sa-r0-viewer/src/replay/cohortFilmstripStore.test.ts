import { describe, expect, it } from "vitest";
import { MAX_FILMSTRIP_SLOTS } from "./cohortFilmstripStore";

describe("cohortFilmstripStore", () => {
  it("caps filmstrip at four slots", () => {
    expect(MAX_FILMSTRIP_SLOTS).toBe(4);
  });
});
