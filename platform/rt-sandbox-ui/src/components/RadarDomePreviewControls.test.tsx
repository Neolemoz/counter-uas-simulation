import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import {
  radarDomeEffectiveRingVisible,
  radarDomeEffectiveVolumeVisible,
  RadarDomeMapQuickControls,
  RadarDomePreviewPanel,
  RadarDomeStatusChips,
} from "./RadarDomePreviewControls";
import {
  radarPreviewLayerActive,
  shouldShowRadarDetectionRing,
  shouldShowRadarVolumePreview,
} from "@/cesium/sensorDomeLayer";

describe("RadarDomePreviewControls", () => {
  const baseState = {
    layerEnabled: true,
    showVolume: true,
    showRing: true,
    selectedOnly: false,
    showLabels: true,
  };

  it("shows ON/OFF status chips", () => {
    const markup = renderToStaticMarkup(
      <RadarDomeStatusChips state={baseState} />,
    );
    expect(markup).toContain("Dome ON");
    expect(markup).toContain("Ring ON");
  });

  it("renders dome preview panel section", () => {
    const markup = renderToStaticMarkup(
      <RadarDomePreviewPanel
        state={baseState}
        handlers={{
          onShowVolumeChange: () => undefined,
          onShowRingChange: () => undefined,
          onSelectedOnlyChange: () => undefined,
          onShowLabelsChange: () => undefined,
        }}
      />,
    );
    expect(markup).toContain("Dome preview");
    expect(markup).toContain("Show radar dome (3D)");
    expect(markup).toContain("Show detection ring");
  });

  it("renders compact map quick controls", () => {
    const markup = renderToStaticMarkup(
      <RadarDomeMapQuickControls
        state={baseState}
        handlers={{
          onShowVolumeChange: () => undefined,
          onShowRingChange: () => undefined,
        }}
      />,
    );
    expect(markup).toContain('data-testid="radar-dome-map-quick-controls"');
    expect(markup).toContain("Dome ON");
    expect(markup).toContain("Ring ON");
  });

  it("derives effective visibility from layer + toggles", () => {
    expect(radarDomeEffectiveVolumeVisible(baseState)).toBe(true);
    expect(
      radarDomeEffectiveVolumeVisible({ ...baseState, showVolume: false }),
    ).toBe(false);
    expect(
      radarDomeEffectiveRingVisible({ ...baseState, layerEnabled: false }),
    ).toBe(false);
  });
});

describe("sensorDome preview gating", () => {
  it("splits ring and volume preview toggles", () => {
    expect(shouldShowRadarDetectionRing({ showRing: false })).toBe(false);
    expect(shouldShowRadarVolumePreview(true, { showVolume: false })).toBe(false);
    expect(shouldShowRadarVolumePreview(false, { showVolume: true })).toBe(false);
    expect(
      radarPreviewLayerActive({ showRing: false, showVolume: true }),
    ).toBe(true);
    expect(
      radarPreviewLayerActive({ showRing: false, showVolume: false }),
    ).toBe(false);
  });
});
