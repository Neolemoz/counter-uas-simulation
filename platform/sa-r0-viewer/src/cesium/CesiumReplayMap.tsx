import { useEffect, useRef } from "react";
import {
  BoundingSphere,
  Cartesian2,
  Cartesian3,
  Color,
  DistanceDisplayCondition,
  EllipsoidTerrainProvider,
  LabelStyle,
  Viewer,
  PolylineDashMaterialProperty,
} from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import type { ReplaySaBundle } from "@/replay/bundleSchema";
import type { LayerVisibility, LosScope } from "@/replay/clockStore";
import { primaryThreatTrackId, useClockStore } from "@/replay/clockStore";
import type { TopologyDiffHighlight } from "@/replay/topologyDiff";
import { enuToCartographic } from "./coordinates";
import { applyTerrainOffset, hasFictionalTerrain } from "./fictionalTerrain";
import { eventFocusPoints, replayBoundingSphere } from "./fitReplay";
import { getLaunchSegment, hasInterceptorSamples } from "./interceptorLaunch";
import { addLosSegmentEntities } from "./losSegmentLayer";
import {
  isOverlayActiveAtT,
  overlayFillColor,
  overlayHighlightOutline,
  overlayOutlineColor,
  OVERLAY_SCENARIO_CAVEAT,
} from "./overlayStyles";
import { splitTrailAndFuture } from "@/replay/trackPlayback";
import { useSweepStore } from "@/replay/useSweepStore";
import { spatialFromBundle, spatialFromSweep } from "@/replay/spatial/spatialAnalytics";
import { syncSpatialGridLayer } from "./spatialGridLayer";
import {
  sortZonesByRadiusDesc,
  zoneDisplayLabel,
  zoneFillColor,
  zoneOutlineColor,
  ZONE_SCENARIO_CAVEAT,
} from "./zoneStyles";

function toCartesian(bundle: ReplaySaBundle, x: number, y: number, z = 0): Cartesian3 {
  const zAdj = hasFictionalTerrain(bundle) ? applyTerrainOffset(bundle, x, y, z, 0.35) : z;
  const c = enuToCartographic(bundle, x, y, zAdj);
  return Cartesian3.fromRadians(c.longitude, c.latitude, c.height);
}

function trackColors(role: string, highlight: boolean) {
  if (highlight) {
    return { trail: "#f472b6", head: Color.MAGENTA, future: "#f472b6" };
  }
  if (role === "threat") {
    return { trail: "#fb923c", head: Color.fromCssColorString("#fb923c"), future: "#fdba74" };
  }
  return { trail: "#22d3ee", head: Color.fromCssColorString("#22d3ee"), future: "#67e8f9" };
}

function linkedIdsForHighlight(
  bundle: ReplaySaBundle,
  selectedEventId: string | null,
): { overlayIds: Set<string>; eventIds: string[] } {
  const eventIds: string[] = selectedEventId ? [selectedEventId] : [];
  const overlayIds = new Set<string>();
  if (!selectedEventId) return { overlayIds, eventIds };
  for (const ov of bundle.overlays) {
    if ((ov.linked_event_ids ?? []).includes(selectedEventId)) {
      overlayIds.add(ov.overlay_id);
    }
  }
  for (const ann of bundle.narrative.annotations ?? []) {
    const linked = (ann.linked_event_ids as string[] | undefined) ?? [];
    if (linked.includes(selectedEventId)) {
      for (const ov of bundle.overlays) {
        if ((ov.linked_event_ids ?? []).some((id) => linked.includes(id))) {
          overlayIds.add(ov.overlay_id);
        }
      }
    }
  }
  return { overlayIds, eventIds };
}

export type MapReplayState = {
  currentT: number;
  layers: LayerVisibility;
  highlightedTrackIds: string[];
  selectedEventId: string | null;
  losScope: LosScope;
  fitReplayNonce: number;
};

type Props = {
  bundle: ReplaySaBundle;
  className?: string;
  onInterceptorSamplesMissing?: (missing: boolean) => void;
  replayState?: MapReplayState;
  diffHighlight?: TopologyDiffHighlight;
  emphasizeDelta?: boolean;
  isCameraLeader?: boolean;
  cameraLocked?: boolean;
  onCameraMatrix?: (state: { position: Cartesian3; direction: Cartesian3; up: Cartesian3 }) => void;
  followCamera?: { position: Cartesian3; direction: Cartesian3; up: Cartesian3 } | null;
};

export function CesiumReplayMap({
  bundle,
  className,
  onInterceptorSamplesMissing,
  replayState,
  diffHighlight,
  emphasizeDelta = false,
  isCameraLeader = false,
  cameraLocked = false,
  onCameraMatrix,
  followCamera,
}: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  const viewerRef = useRef<Viewer | null>(null);
  const clockCurrentT = useClockStore((s) => s.currentT);
  const clockLayers = useClockStore((s) => s.layers);
  const clockHighlighted = useClockStore((s) => s.highlightedTrackIds);
  const clockSelected = useClockStore((s) => s.selectedEventId);
  const clockLosScope = useClockStore((s) => s.losScope);
  const clockFitNonce = useClockStore((s) => s.fitReplayNonce);
  const activeSweep = useSweepStore((s) => s.sweep);

  const currentT = replayState?.currentT ?? clockCurrentT;
  const layers = replayState?.layers ?? clockLayers;
  const highlightedTrackIds = replayState?.highlightedTrackIds ?? clockHighlighted;
  const selectedEventId = replayState?.selectedEventId ?? clockSelected;
  const losScope = replayState?.losScope ?? clockLosScope;
  const fitReplayNonce = replayState?.fitReplayNonce ?? clockFitNonce;

  const deltaEntities = new Set(diffHighlight?.entityIds ?? []);
  const deltaOverlays = new Set(diffHighlight?.overlayIds ?? []);
  const deltaZones = new Set(diffHighlight?.zoneIds ?? []);

  useEffect(() => {
    onInterceptorSamplesMissing?.(!hasInterceptorSamples(bundle));
  }, [bundle, onInterceptorSamplesMissing]);

  useEffect(() => {
    if (!containerRef.current) return;
    const viewer = new Viewer(containerRef.current, {
      terrainProvider: new EllipsoidTerrainProvider(),
      animation: false,
      timeline: false,
      baseLayerPicker: false,
      geocoder: false,
      homeButton: false,
      sceneModePicker: true,
      navigationHelpButton: false,
      fullscreenButton: true,
    });
    viewer.scene.globe.enableLighting = true;
    const imagery = viewer.scene.imageryLayers.get(0);
    if (imagery) {
      imagery.alpha = 0.88;
    }
    viewerRef.current = viewer;
    return () => {
      viewer.destroy();
      viewerRef.current = null;
    };
  }, []);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer) return;
    viewer.entities.removeAll();

    const { overlayIds: highlightedOverlays, eventIds: highlightEvents } =
      linkedIdsForHighlight(bundle, selectedEventId);

    if (layers.zones) {
      const zones = sortZonesByRadiusDesc(bundle.zones);
      for (const zone of zones) {
        const g = zone.geometry;
        if (g.type !== "circle" || !g.center_enu_m || !g.radius_m) continue;
        const [cx, cy, cz = 0] = g.center_enu_m;
        const center = toCartesian(bundle, cx, cy, cz);
        const label = zone.display_label ?? zoneDisplayLabel(zone.zone_id);
        const zoneDelta = deltaZones.has(zone.zone_id);
        viewer.entities.add({
          id: zone.zone_id,
          position: center,
          ellipse: {
            semiMajorAxis: g.radius_m,
            semiMinorAxis: g.radius_m,
            material: zoneFillColor(zone.zone_id, layers.overlays).withAlpha(
              emphasizeDelta && !zoneDelta && deltaZones.size > 0 ? 0.2 : 1,
            ),
            outline: true,
            outlineColor: zoneDelta
              ? Color.fromCssColorString("#f59e0b")
              : zoneOutlineColor(zone.zone_id),
            outlineWidth: zoneDelta ? 2 : 1,
            height: cz,
          },
          description: `${label}<br/>${zone.caveat ?? ZONE_SCENARIO_CAVEAT}`,
        });
        viewer.entities.add({
          id: `${zone.zone_id}_label`,
          position: toCartesian(bundle, cx, cy + g.radius_m * 0.85, cz),
          label: {
            text: label,
            font: "12px sans-serif",
            fillColor: Color.WHITE,
            outlineColor: Color.BLACK,
            outlineWidth: 2,
            style: LabelStyle.FILL,
            showBackground: true,
            backgroundColor: Color.BLACK.withAlpha(0.55),
            scale: 0.9,
            distanceDisplayCondition: new DistanceDisplayCondition(0, 8_000_000),
          },
        });
      }
    }

    if (layers.overlays) {
      for (const ov of bundle.overlays) {
        if (!isOverlayActiveAtT(ov.active_t_range, currentT)) continue;
        const verts = ov.geometry.vertices_enu_m;
        if (!verts?.length) continue;
        const highlight =
          highlightedOverlays.has(ov.overlay_id) || deltaOverlays.has(ov.overlay_id);
        const positions = verts.map(([x, y, z = 0]) => toCartesian(bundle, x, y, z));
        viewer.entities.add({
          id: ov.overlay_id,
          polygon: {
            hierarchy: positions,
            material: overlayFillColor(ov.kind).withAlpha(
              emphasizeDelta && !deltaOverlays.has(ov.overlay_id) && deltaOverlays.size > 0
                ? 0.15
                : 1,
            ),
            outline: true,
            outlineColor: highlight
              ? overlayHighlightOutline(ov.kind)
              : overlayOutlineColor(ov.kind),
            outlineWidth: highlight ? 3 : 1.5,
          },
          description: `${ov.caveat ?? OVERLAY_SCENARIO_CAVEAT}<br/><em>Explanatory overlay only.</em>`,
        });
        const ridgeLine = ov.geometry.ridge_outline_enu_m;
        if (ridgeLine && ridgeLine.length >= 2 && ov.kind === "ridge_mask") {
          viewer.entities.add({
            id: `${ov.overlay_id}_ridge_outline`,
            polyline: {
              positions: ridgeLine.map(([x, y, z = 0]) => toCartesian(bundle, x, y, z + 5)),
              width: highlight ? 4 : 2.5,
              material: Color.fromCssColorString("#d6d3d1").withAlpha(0.9),
            },
          });
        }
      }
    }

    if (layers.losLinks) {
      addLosSegmentEntities(
        viewer,
        bundle,
        (x, y, z) => toCartesian(bundle, x, y, z),
        currentT,
        selectedEventId,
        highlightEvents,
        {
          losScope,
          trackId: primaryThreatTrackId(bundle),
        },
      );
    }

    if (layers.sites) {
      for (const ent of bundle.entities_static) {
        const [x, y, z] = ent.position_enu_m;
        const isBase = ent.kind === "interceptor_base";
        const isDelta = deltaEntities.has(ent.entity_id);
        const muted = emphasizeDelta && !isDelta && deltaEntities.size > 0;
        viewer.entities.add({
          id: ent.entity_id,
          position: toCartesian(bundle, x, y, z),
          point: {
            pixelSize: isDelta ? 10 : isBase ? 7 : 6,
            color: Color.fromCssColorString(
              isDelta ? "#fbbf24" : isBase ? "#94a3b8" : "#38bdf8",
            ).withAlpha(muted ? 0.35 : 0.85),
            outlineColor: isDelta
              ? Color.fromCssColorString("#f59e0b")
              : Color.WHITE.withAlpha(0.6),
            outlineWidth: isDelta ? 2 : 1,
          },
          label: {
            text: ent.label,
            font: "10px sans-serif",
            fillColor: Color.WHITE,
            style: LabelStyle.FILL,
            pixelOffset: new Cartesian2(0, -12),
            show: false,
            showBackground: true,
            backgroundColor: Color.BLACK.withAlpha(0.65),
            distanceDisplayCondition: new DistanceDisplayCondition(0, 120_000),
          },
          description: `<strong>${ent.label}</strong><br/>Replay fixture — not authoritative.`,
        });
      }
    }

    const launch = getLaunchSegment(bundle, currentT);
    if (layers.tracks && launch) {
      viewer.entities.add({
        id: "interceptor_launch_segment",
        polyline: {
          positions: [
            toCartesian(bundle, launch.base[0], launch.base[1], launch.base[2]),
            toCartesian(
              bundle,
              launch.firstSample.x_m,
              launch.firstSample.y_m,
              launch.firstSample.z_m,
            ),
          ],
          width: 2,
          material: new PolylineDashMaterialProperty({
            color: Color.fromCssColorString("#22d3ee").withAlpha(0.7),
            dashLength: 8,
          }),
        },
      });
    }

    if (layers.tracks) {
      for (const track of bundle.tracks) {
        const { trail, future, head } = splitTrailAndFuture(track.samples, currentT);
        if (!head && trail.length === 0) continue;

        const highlight = highlightedTrackIds.includes(track.track_id);
        const colors = trackColors(track.role, highlight);

        const trailPositions = trail.map((s) =>
          toCartesian(bundle, s.x_m, s.y_m, s.z_m ?? 0),
        );
        if (head) {
          trailPositions.push(toCartesian(bundle, head.x_m, head.y_m, head.z_m));
        }
        if (trailPositions.length >= 2) {
          viewer.entities.add({
            id: track.track_id,
            polyline: {
              positions: trailPositions,
              width: highlight ? 4 : 2.5,
              material: new PolylineDashMaterialProperty({
                color: Color.fromCssColorString(colors.trail).withAlpha(0.9),
                dashLength: 10,
              }),
            },
          });
        }

        if (future.length > 0) {
          const futurePositions = [
            head
              ? toCartesian(bundle, head.x_m, head.y_m, head.z_m)
              : toCartesian(bundle, future[0].x_m, future[0].y_m, future[0].z_m ?? 0),
            ...future.map((s) => toCartesian(bundle, s.x_m, s.y_m, s.z_m ?? 0)),
          ];
          if (futurePositions.length >= 2) {
            viewer.entities.add({
              id: `${track.track_id}_future`,
              polyline: {
                positions: futurePositions,
                width: 1.5,
                material: new PolylineDashMaterialProperty({
                  color: Color.fromCssColorString(colors.future).withAlpha(0.35),
                  dashLength: 16,
                }),
              },
            });
          }
        }

        if (head) {
          viewer.entities.add({
            id: `${track.track_id}_head`,
            position: toCartesian(bundle, head.x_m, head.y_m, head.z_m),
            point: {
              pixelSize: highlight ? 14 : 10,
              color: colors.head,
              outlineColor: Color.WHITE,
              outlineWidth: highlight ? 2 : 1,
            },
            label: highlight
              ? {
                  text: track.track_id,
                  font: "11px sans-serif",
                  fillColor: Color.WHITE,
                  style: LabelStyle.FILL,
                  pixelOffset: new Cartesian2(0, -16),
                  showBackground: true,
                  backgroundColor: Color.BLACK.withAlpha(0.7),
                }
              : undefined,
          });
        }
      }
    }

    if (layers.narrativeMarkers) {
      for (const ev of bundle.narrative.events) {
        const lineIndex = ev.line_index as number | null | undefined;
        if (lineIndex == null || lineIndex > currentT) continue;
        let pos: Cartesian3 | null = null;
        for (const track of bundle.tracks) {
          const { head } = splitTrailAndFuture(track.samples, lineIndex);
          if (head) {
            pos = toCartesian(bundle, head.x_m, head.y_m, head.z_m);
            break;
          }
        }
        if (!pos) continue;
        const isSelected = ev.event_id === selectedEventId;
        viewer.entities.add({
          id: String(ev.event_id),
          position: pos,
          point: {
            pixelSize: isSelected ? 12 : 5,
            color: isSelected ? Color.YELLOW : Color.ORANGE.withAlpha(0.8),
          },
        });
      }
    }

    const spatialLayers = activeSweep
      ? spatialFromSweep(activeSweep)
      : spatialFromBundle(bundle);
    const spatialGrid = activeSweep?.spatial_aggregate.grid ?? bundle.spatial_analytics?.grid;
    const declutter = useSweepStore.getState().spatialDeclutter;
    if (spatialLayers && spatialGrid && layers.spatial) {
      syncSpatialGridLayer(
        viewer,
        bundle,
        spatialGrid,
        spatialLayers,
        layers.spatial,
        "spatial-grid",
        activeSweep ? declutter : "off",
      );
    }
  }, [
    bundle,
    currentT,
    layers,
    highlightedTrackIds,
    selectedEventId,
    losScope,
    diffHighlight,
    emphasizeDelta,
    activeSweep,
  ]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !isCameraLeader || !onCameraMatrix) return;
    const handler = () => {
      const cam = viewer.camera;
      onCameraMatrix({
        position: cam.position.clone(),
        direction: cam.direction.clone(),
        up: cam.up.clone(),
      });
    };
    viewer.camera.changed.addEventListener(handler);
    return () => {
      viewer.camera.changed.removeEventListener(handler);
    };
  }, [isCameraLeader, onCameraMatrix]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !followCamera || !cameraLocked) return;
    viewer.camera.setView({
      destination: followCamera.position,
      orientation: {
        direction: followCamera.direction,
        up: followCamera.up,
      },
    });
  }, [followCamera, cameraLocked]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer) return;
    const sphere = replayBoundingSphere(bundle);
    if (sphere) {
      viewer.camera.flyToBoundingSphere(sphere, { duration: 0.5 });
    }
  }, [bundle, fitReplayNonce]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !selectedEventId) return;
    const ev = bundle.narrative.events.find((e) => e.event_id === selectedEventId);
    const lineIndex = ev?.line_index as number | undefined;
    if (lineIndex == null) return;
    const points = eventFocusPoints(bundle, lineIndex);
    if (points.length === 0) return;
    const sphere = BoundingSphere.fromPoints(points);
    viewer.camera.flyToBoundingSphere(sphere, { duration: 0.6 });
  }, [bundle, selectedEventId]);

  return <div ref={containerRef} className={className ?? "h-full w-full min-h-[320px]"} />;
}
