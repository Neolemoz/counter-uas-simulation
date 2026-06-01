import { describe, expect, it } from "vitest";
import {
  coerceSensorDomeRadii,
  countEntitiesInNominalDome,
  horizontalDistanceM,
  labelAzimuthDegForRadarDetection,
  labelAzimuthDegForRadarVertical,
  labelRadiusScaleForRadarDetection,
  labelRadiusScaleForRadarVertical,
  normalizedRadarDomeConfig,
  radarDetectionLabelText,
  radarVerticalCoverageLabelText,
  RADAR_RING_ALPHA_DEFAULT,
  RADAR_RING_ALPHA_SELECTED,
  RADAR_VOLUME_FILL_ALPHA,
  RADAR_VOLUME_OUTLINE_ALPHA,
  RADAR_VOLUME_OUTLINE_WIDTH,
  radarRingFade,
  shouldRenderRadarRing,
  shouldRenderRadarVolumeDome,
  shouldShowRadarDetectionRing,
  shouldShowRadarVolumePreview,
  radarPreviewLayerActive,
} from "./sensorDomeLayer";
import type { MirrorEntity } from "./entityMarkers";

describe("sensorDomeLayer", () => {
  it("computes horizontal distance", () => {
    expect(horizontalDistanceM(0, 0, 3, 4)).toBe(5);
  });

  it("counts entities inside nominal dome", () => {
    const entities: MirrorEntity[] = [
      { entity_id: "r1", entity_type: "radar", pose: { x: 0, y: 0, z: 10 } },
      { entity_id: "d1", entity_type: "drone", pose: { x: 50, y: 0, z: 10 } },
      { entity_id: "d2", entity_type: "drone", pose: { x: 400, y: 0, z: 10 } },
    ];
    expect(countEntitiesInNominalDome(0, 0, entities, 200)).toBe(1);
  });

  it("uses distinct label bearings and radii for horizontal and vertical coverage", () => {
    const detection = labelAzimuthDegForRadarDetection();
    const vertical = labelAzimuthDegForRadarVertical();
    expect(detection).toBeGreaterThan(0);
    expect(vertical).not.toBe(detection);
    expect(Math.abs(detection - vertical)).toBeGreaterThanOrEqual(80);
    expect(labelRadiusScaleForRadarVertical()).toBeGreaterThan(
      labelRadiusScaleForRadarDetection(),
    );
  });

  it("formats detection radius label separately from vertical coverage", () => {
    expect(radarDetectionLabelText(300)).toBe("Detection · 300m");
    expect(radarVerticalCoverageLabelText()).toBe("z 0–200m");
  });

  it("describes scenario vertical coverage for selected radar labels", () => {
    expect(radarVerticalCoverageLabelText()).toBe("z 0–200m");
  });

  it("shows 3D volume preview only for the selected radar", () => {
    expect(shouldRenderRadarVolumeDome(true)).toBe(true);
    expect(shouldRenderRadarVolumeDome(false)).toBe(false);
    expect(shouldShowRadarVolumePreview(true, { showVolume: false })).toBe(false);
  });

  it("coerces legacy outer radius into detection config", () => {
    const radii = coerceSensorDomeRadii({ innerM: 40, middleM: 120, outerM: 260 });
    expect(radii.detectionM).toBe(260);
  });

  it("normalizes radar detection radius floor", () => {
    const radii = normalizedRadarDomeConfig({ detectionM: 3 });
    expect(radii.detectionM).toBeGreaterThanOrEqual(10);
  });

  it("renders all radar rings when selected-radar-only is off", () => {
    expect(shouldRenderRadarRing("radar", "r1", "r2", false)).toBe(true);
    expect(shouldRenderRadarRing("radar", "r2", "r2", false)).toBe(true);
    expect(shouldRenderRadarRing("radar", "r1", "r2", true)).toBe(false);
    expect(shouldRenderRadarRing("radar", "r2", "r2", true)).toBe(true);
  });

  it("keeps non-selected radar rings visibly softer than selected", () => {
    expect(radarRingFade(true, true, true)).toBe(1);
    expect(radarRingFade(false, true, true)).toBeGreaterThan(0.5);
    expect(radarRingFade(false, true, true)).toBeLessThan(1);
    expect(radarRingFade(false, false, true)).toBeGreaterThan(
      radarRingFade(false, true, true),
    );
  });

  it("keeps radar ring opacity below defense zone emphasis", () => {
    expect(RADAR_RING_ALPHA_SELECTED).toBeLessThan(0.96);
    expect(RADAR_RING_ALPHA_DEFAULT).toBeLessThan(RADAR_RING_ALPHA_SELECTED);
    expect(RADAR_RING_ALPHA_SELECTED).toBeGreaterThan(0.5);
  });

  it("keeps 3D volume fill softer than outline for topo contrast", () => {
    expect(RADAR_VOLUME_FILL_ALPHA).toBeLessThan(0.12);
    expect(RADAR_VOLUME_OUTLINE_ALPHA).toBeGreaterThan(RADAR_VOLUME_FILL_ALPHA);
    expect(RADAR_VOLUME_OUTLINE_WIDTH).toBeGreaterThanOrEqual(2);
  });
});
