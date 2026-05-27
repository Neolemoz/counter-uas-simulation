import { describe, expect, it } from "vitest";
import type { Viewer } from "cesium";
import {
  attachCesiumEditingHandlers,
  isViewerUsable,
  restoreCamera,
} from "./cesiumEditing";
import { parseRtEntityId } from "./entityId";

function fakeViewer(): Viewer {
  return {
    isDestroyed: () => false,
    scene: {
      screenSpaceCameraController: {
        enableRotate: false,
        enableTranslate: false,
        enableZoom: false,
        enableTilt: false,
        enableLook: false,
      },
    },
    camera: {},
    entities: {},
  } as unknown as Viewer;
}

function destroyedViewer(): Viewer {
  return {
    isDestroyed: () => true,
    get scene() {
      throw new Error("destroyed scene accessed");
    },
  } as unknown as Viewer;
}

describe("cesiumEditing pick helpers", () => {
  it("extracts entity id from cesium pick id shape", () => {
    const cesiumId = "rt-entity-550e8400-e29b-41d4-a716-446655440000";
    expect(parseRtEntityId(cesiumId)).toBe(
      "550e8400-e29b-41d4-a716-446655440000",
    );
  });
});

describe("cesium viewer lifecycle guards", () => {
  it("treats undefined viewer during connect as unusable", () => {
    expect(isViewerUsable(undefined)).toBe(false);
    expect(() => attachCesiumEditingHandlers(undefined, {
      editingEnabled: true,
      selectedType: "drone",
      worldSummary: undefined,
      onSelectEntity: () => undefined,
      onSpawn: () => undefined,
      onMove: () => undefined,
    })()).not.toThrow();
  });

  it("treats null viewer after teardown as unusable", () => {
    expect(isViewerUsable(null)).toBe(false);
    expect(() => restoreCamera(null)).not.toThrow();
  });

  it("keeps restoreCamera safe before init or after destroy", () => {
    expect(() => restoreCamera(undefined)).not.toThrow();
    expect(() => restoreCamera(destroyedViewer())).not.toThrow();
  });

  it("restores camera controls when a usable viewer exists", () => {
    const viewer = fakeViewer();
    restoreCamera(viewer);

    const controller = viewer.scene.screenSpaceCameraController;
    expect(controller.enableRotate).toBe(true);
    expect(controller.enableTranslate).toBe(true);
    expect(controller.enableZoom).toBe(true);
    expect(controller.enableTilt).toBe(true);
    expect(controller.enableLook).toBe(true);
  });

  it("allows connect, disconnect, and connect again without reusing stale viewers", () => {
    let current: Viewer | null = null;
    const acceptViewer = (nextViewer: Viewer | null | undefined) => {
      current = isViewerUsable(nextViewer) ? nextViewer : null;
    };

    const first = fakeViewer();
    const second = fakeViewer();
    acceptViewer(first);
    expect(current).toBe(first);

    acceptViewer(null);
    expect(current).toBeNull();

    acceptViewer(second);
    expect(current).toBe(second);
  });
});
