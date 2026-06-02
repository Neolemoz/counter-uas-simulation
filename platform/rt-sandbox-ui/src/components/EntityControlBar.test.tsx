import { beforeEach, describe, expect, it, vi } from "vitest";
import { renderToStaticMarkup } from "react-dom/server";
import { deleteEntity, spawnAttacker, spawnEntity } from "@/bridge/entityCommands";
import { ENTITY_CONTROL_GOVERNANCE } from "@/entity/entityControlStates";
import { EntityControlBar } from "./RefreshControls";

const handlers = {
  onSpawnAttacker: () => {},
  onSpawnDefender: () => {},
  onDeleteSelected: () => {},
};

describe("EntityControlBar", () => {
  it("renders nothing when not connected", () => {
    const markup = renderToStaticMarkup(
      <EntityControlBar
        connected={false}
        controlsDisabled
        deleteDisabled
        selectedEntityId={null}
        {...handlers}
      />,
    );
    expect(markup).toBe("");
  });

  it("renders governance and controls when connected", () => {
    const markup = renderToStaticMarkup(
      <EntityControlBar
        connected
        controlsDisabled={false}
        deleteDisabled
        selectedEntityId="e-1"
        {...handlers}
      />,
    );
    expect(markup).toContain('data-testid="entity-control-bar"');
    expect(markup).toContain(ENTITY_CONTROL_GOVERNANCE);
    expect(markup).toContain("Spawn attacker");
    expect(markup).toContain("Spawn defender");
    expect(markup).toContain("e-1");
  });

  it("disables spawn controls when controlsDisabled", () => {
    const markup = renderToStaticMarkup(
      <EntityControlBar
        connected
        controlsDisabled
        deleteDisabled
        selectedEntityId="e-1"
        {...handlers}
      />,
    );
    const attacker = markup.match(/<button[^>]*data-testid="entity-spawn-attacker"[^>]*>/)?.[0] ?? "";
    const defender = markup.match(/<button[^>]*data-testid="entity-spawn-defender"[^>]*>/)?.[0] ?? "";
    expect(attacker).toMatch(/\sdisabled(?:=""|(?=\s|>))/);
    expect(defender).toMatch(/\sdisabled(?:=""|(?=\s|>))/);
  });

  it("disables delete when deleteDisabled", () => {
    const markup = renderToStaticMarkup(
      <EntityControlBar
        connected
        controlsDisabled={false}
        deleteDisabled
        selectedEntityId={null}
        {...handlers}
      />,
    );
    const deleteBtn = markup.match(/<button[^>]*data-testid="entity-delete-selected"[^>]*>/)?.[0] ?? "";
    expect(deleteBtn).toMatch(/\sdisabled(?:=""|(?=\s|>))/);
  });
});

describe("entity command dispatch", () => {
  beforeEach(() => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK", entity_id: "new-1" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });
  });

  it("spawn_attacker for drone path", async () => {
    await spawnAttacker("sid", { pose: { x: 0, y: 0, z: 10, yaw_deg: 0 } });
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("spawn_attacker");
    expect(body.session_id).toBe("sid");
  });

  it("spawn_entity for interceptor path", async () => {
    await spawnEntity("sid", {
      entity_type: "interceptor",
      pose: { x: 0, y: 0, z: 10, yaw_deg: 0 },
    });
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("spawn_entity");
    expect(body.payload.entity_type).toBe("interceptor");
  });

  it("delete_entity for selected delete path", async () => {
    await deleteEntity("sid", "e-42");
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("delete_entity");
    expect(body.payload.entity_id).toBe("e-42");
  });
});
