import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { promptJsonImport } from "./useJsonPromptImport";

function stubWindow(prompt: ReturnType<typeof vi.fn>, alert: ReturnType<typeof vi.fn>) {
  vi.stubGlobal("window", { prompt, alert });
}

describe("promptJsonImport", () => {
  let promptFn: ReturnType<typeof vi.fn>;
  let alertFn: ReturnType<typeof vi.fn>;

  beforeEach(() => {
    promptFn = vi.fn();
    alertFn = vi.fn();
    stubWindow(promptFn, alertFn);
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("returns early when prompt is cancelled", () => {
    promptFn.mockReturnValue(null);
    const parse = vi.fn();
    const onSuccess = vi.fn();

    promptJsonImport({
      promptMessage: "Paste JSON",
      invalidLabel: "manifest",
      parse,
      onSuccess,
    });

    expect(parse).not.toHaveBeenCalled();
    expect(onSuccess).not.toHaveBeenCalled();
    expect(alertFn).not.toHaveBeenCalled();
  });

  it("returns early when prompt is empty", () => {
    promptFn.mockReturnValue("");
    const parse = vi.fn();
    const onSuccess = vi.fn();

    promptJsonImport({
      promptMessage: "Paste JSON",
      invalidLabel: "manifest",
      parse,
      onSuccess,
    });

    expect(parse).not.toHaveBeenCalled();
    expect(onSuccess).not.toHaveBeenCalled();
  });

  it("alerts on parse failure without calling onSuccess", () => {
    promptFn.mockReturnValue("{");
    const onSuccess = vi.fn();

    promptJsonImport({
      promptMessage: "Paste rt_experiment_manifest_v1 JSON",
      invalidLabel: "manifest",
      parse: () => ({ ok: false, error: "bad json" }),
      onSuccess,
    });

    expect(alertFn).toHaveBeenCalledWith("Invalid manifest: bad json");
    expect(onSuccess).not.toHaveBeenCalled();
  });

  it("truncates long parse errors in alert", () => {
    const longError = "x".repeat(150);
    promptFn.mockReturnValue("{}");

    promptJsonImport({
      promptMessage: "Paste JSON",
      invalidLabel: "metrics report",
      parse: () => ({ ok: false, error: longError }),
      onSuccess: vi.fn(),
    });

    expect(alertFn).toHaveBeenCalledWith(
      `Invalid metrics report: ${"x".repeat(117)}...`,
    );
  });

  it("calls onSuccess with parsed data", () => {
    const data = { schema: "rt_experiment_manifest_v1" };
    const onSuccess = vi.fn();
    promptFn.mockReturnValue("{}");

    promptJsonImport({
      promptMessage: "Paste rt_experiment_manifest_v1 JSON",
      invalidLabel: "manifest",
      parse: () => ({ ok: true, data }),
      onSuccess,
    });

    expect(onSuccess).toHaveBeenCalledWith(data);
    expect(alertFn).not.toHaveBeenCalled();
  });
});
