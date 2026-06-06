import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { ProtectedCenterRecoveryBanner } from "./ProtectedCenterRecoveryBanner";
import {
  PROTECTED_CENTER_CLEARED_RESET_COPY,
  PROTECTED_CENTER_REDESIGNATE_COPY,
  protectedCenterRecoveryMessage,
} from "./protectedCenterCopy";

describe("ProtectedCenterRecoveryBanner", () => {
  it("shows reset recovery guidance when designation was cleared", () => {
    const markup = renderToStaticMarkup(
      <ProtectedCenterRecoveryBanner
        recoveryNotice="reset_session"
        protectedCenterEntityId={null}
      />,
    );
    expect(markup).toContain('data-testid="protected-center-recovery-banner"');
    expect(markup).toContain(PROTECTED_CENTER_CLEARED_RESET_COPY);
    expect(markup).toContain(PROTECTED_CENTER_REDESIGNATE_COPY);
  });

  it("hides when a protected center is still designated", () => {
    const markup = renderToStaticMarkup(
      <ProtectedCenterRecoveryBanner
        recoveryNotice="reset_session"
        protectedCenterEntityId="center-a"
      />,
    );
    expect(markup).toBe("");
  });

  it("shows unavailable recovery copy", () => {
    expect(protectedCenterRecoveryMessage("protected_center_unavailable")).toContain(
      "no longer valid",
    );
  });
});
