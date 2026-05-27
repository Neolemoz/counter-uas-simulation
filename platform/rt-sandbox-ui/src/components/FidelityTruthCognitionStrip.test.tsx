import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { FidelityTruthCognitionStrip } from "./FidelityTruthCognitionStrip";
import { extractFidelityContext } from "@/fidelity/fidelityCognition";
import { BANNER_FIDELITY_TRUTH } from "@/governance/banners";
import { containsForbiddenLexicon } from "@/cesium/cognition";

describe("FidelityTruthCognitionStrip", () => {
  it("renders nothing when coupling is off", () => {
    const markup = renderToStaticMarkup(
      <FidelityTruthCognitionStrip
        fidelityContext={extractFidelityContext({
          enable_fidelity_coupling: false,
        })}
      />,
    );
    expect(markup).toBe("");
  });

  it("renders truth strip with banner and badges when coupling on", () => {
    const markup = renderToStaticMarkup(
      <FidelityTruthCognitionStrip
        fidelityContext={extractFidelityContext({
          enable_fidelity_coupling: true,
          fidelity_attestation_status: "available",
          fidelity_label: "truth_attested",
          fidelity_truth: {
            timestamp_utc: "2026-05-26T16:00:00+00:00",
            los_truth: { label: "clear", pair_entity_ids: ["a"] },
            dome_truth: { sensor_id: "radar_north", entities_in_nominal_dome: 1 },
            entity_truth: [
              {
                entity_id: "e1",
                truth_attested_pose: { x: 1, y: 2, z: 3 },
                sim_agl_m: 3,
              },
            ],
          },
        })}
        worldSummary={{
          entities: [{ entity_id: "e1", pose: { x: 1.5, y: 2, z: 3 } }],
        }}
      />,
    );
    expect(markup).toContain('data-testid="fidelity-truth-cognition-strip"');
    expect(markup).toContain(BANNER_FIDELITY_TRUTH);
    expect(markup).toContain("LOS truth");
    expect(markup).toContain("pose_truth_drift");
  });

  it("avoids forbidden lexicon in banner", () => {
    expect(containsForbiddenLexicon(BANNER_FIDELITY_TRUTH)).toBe(false);
  });
});
