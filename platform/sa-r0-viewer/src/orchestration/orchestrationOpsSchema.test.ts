import { describe, expect, it } from "vitest";
import {
  orchestrationIntegrityReportSchema,
  orchestrationOpsManifestSchema,
} from "./orchestrationOpsSchema";

describe("orchestrationOpsManifestSchema", () => {
  it("parses sample ops sidecar fields", () => {
    const sample = {
      artifact_type: "experiment_orchestration_ops_manifest_v1",
      schema_version: "1",
      manifest_id: "valley_ingress_validation_only",
      operations_status: "queued",
      scenario_pack_ids: ["valley_ingress"],
    };
    const parsed = orchestrationOpsManifestSchema.parse(sample);
    expect(parsed.operations_status).toBe("queued");
  });
});

describe("orchestrationIntegrityReportSchema", () => {
  it("parses minimal integrity report", () => {
    const sample = {
      artifact_type: "orchestration_integrity_report_v1",
      schema_version: "1",
      ok: true,
      pack_count: 12,
      manifest_count: 12,
    };
    const parsed = orchestrationIntegrityReportSchema.parse(sample);
    expect(parsed.ok).toBe(true);
  });
});
