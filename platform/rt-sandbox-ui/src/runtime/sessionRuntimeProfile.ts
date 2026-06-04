/** Session runtime profile selection for start_session (PLAT-RT-ADAPTER-R1 A2 + live Step 2). */

export type SessionRuntimeProfile = "stub" | "mock_adapter" | "live";

export const DEFAULT_SESSION_RUNTIME_PROFILE: SessionRuntimeProfile = "stub";

export const SESSION_RUNTIME_PROFILE_STORAGE_KEY =
  "rt_sandbox_session_runtime_profile_v1";

export const MOCK_ADAPTER_GOVERNANCE =
  "Mock adapter — simulation-only in-process worker. Not live Gazebo. Not ROS-backed.";

export const STUB_RUNTIME_GOVERNANCE =
  "Stub runtime — bridge RuntimeStub with no adapter worker.";

export const LIVE_GAZEBO_GOVERNANCE =
  "Live Gazebo — experimental local-host runtime. Requires ROS 2 + Gazebo + rt_sandbox_gz on PATH. Not operational state.";

export type RuntimeProfileOption = {
  id: SessionRuntimeProfile;
  title: string;
  description: string;
  governance: string;
};

export const SESSION_RUNTIME_PROFILE_OPTIONS: RuntimeProfileOption[] = [
  {
    id: "stub",
    title: "Stub runtime",
    description: "Default bridge stub — no adapter subprocess.",
    governance: STUB_RUNTIME_GOVERNANCE,
  },
  {
    id: "mock_adapter",
    title: "Mock adapter",
    description: "GazeboRuntimeAdapter in mock mode — in-memory sim feedback only.",
    governance: MOCK_ADAPTER_GOVERNANCE,
  },
  {
    id: "live",
    title: "Live Gazebo",
    description: "GazeboRuntimeAdapter in live mode — launches rt_sandbox_gz on local host.",
    governance: LIVE_GAZEBO_GOVERNANCE,
  },
];

export function isSessionRuntimeProfile(value: unknown): value is SessionRuntimeProfile {
  return value === "stub" || value === "mock_adapter" || value === "live";
}

export function readStoredSessionRuntimeProfile(): SessionRuntimeProfile {
  if (typeof localStorage === "undefined") return DEFAULT_SESSION_RUNTIME_PROFILE;
  try {
    const raw = localStorage.getItem(SESSION_RUNTIME_PROFILE_STORAGE_KEY);
    return isSessionRuntimeProfile(raw) ? raw : DEFAULT_SESSION_RUNTIME_PROFILE;
  } catch {
    return DEFAULT_SESSION_RUNTIME_PROFILE;
  }
}

export function writeStoredSessionRuntimeProfile(profile: SessionRuntimeProfile): void {
  if (typeof localStorage === "undefined") return;
  try {
    localStorage.setItem(SESSION_RUNTIME_PROFILE_STORAGE_KEY, profile);
  } catch {
    /* ignore quota / privacy mode */
  }
}

export function buildStartSessionPayload(
  profile: SessionRuntimeProfile,
): Record<string, string> | undefined {
  if (profile === DEFAULT_SESSION_RUNTIME_PROFILE) {
    return undefined;
  }
  return { runtime_profile: profile };
}

export function sessionRuntimeProfileLabel(profile: SessionRuntimeProfile): string {
  return (
    SESSION_RUNTIME_PROFILE_OPTIONS.find((opt) => opt.id === profile)?.title ?? profile
  );
}

export function sessionRuntimeProfileGovernance(profile: SessionRuntimeProfile): string {
  return (
    SESSION_RUNTIME_PROFILE_OPTIONS.find((opt) => opt.id === profile)?.governance ?? ""
  );
}
