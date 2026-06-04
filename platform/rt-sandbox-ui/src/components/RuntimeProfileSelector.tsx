import {
  DEFAULT_SESSION_RUNTIME_PROFILE,
  LIVE_GAZEBO_GOVERNANCE,
  MOCK_ADAPTER_GOVERNANCE,
  SESSION_RUNTIME_PROFILE_OPTIONS,
  type SessionRuntimeProfile,
} from "@/runtime/sessionRuntimeProfile";

export function RuntimeProfileSelector({
  value,
  onChange,
  disabled = false,
  showGovernance = true,
}: {
  value: SessionRuntimeProfile;
  onChange: (profile: SessionRuntimeProfile) => void;
  disabled?: boolean;
  showGovernance?: boolean;
}) {
  const selected = SESSION_RUNTIME_PROFILE_OPTIONS.find((opt) => opt.id === value);

  return (
    <fieldset
      className="space-y-2 rounded-lg border border-slate-700/80 bg-slate-900/50 px-3 py-2"
      data-testid="runtime-profile-selector"
      disabled={disabled}
    >
      <legend className="px-1 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Session runtime
      </legend>
      <div className="flex flex-col gap-2 sm:flex-row sm:flex-wrap">
        {SESSION_RUNTIME_PROFILE_OPTIONS.map((opt) => {
          const checked = value === opt.id;
          const accent =
            opt.id === "live"
              ? checked
                ? "border-emerald-700/70 bg-emerald-950/40"
                : "border-slate-800 bg-slate-950/40 hover:border-slate-600"
              : opt.id === "mock_adapter"
                ? checked
                  ? "border-cyan-700/70 bg-cyan-950/40"
                  : "border-slate-800 bg-slate-950/40 hover:border-slate-600"
                : checked
                  ? "border-slate-500/80 bg-slate-800/60"
                  : "border-slate-800 bg-slate-950/40 hover:border-slate-600";
          return (
            <label
              key={opt.id}
              className={`flex min-w-[10rem] flex-1 cursor-pointer flex-col rounded border px-3 py-2 text-xs transition-colors ${accent} ${disabled ? "cursor-not-allowed opacity-50" : ""}`}
              data-testid={`runtime-profile-option-${opt.id}`}
            >
              <span className="flex items-center gap-2">
                <input
                  type="radio"
                  name="session_runtime_profile"
                  value={opt.id}
                  checked={checked}
                  disabled={disabled}
                  onChange={() => onChange(opt.id)}
                  className="rounded-full"
                />
                <span className="font-semibold text-slate-200">{opt.title}</span>
              </span>
              <span className="mt-1 text-slate-500">{opt.description}</span>
            </label>
          );
        })}
      </div>
      {showGovernance && selected && (
        <p className="text-[11px] leading-snug text-slate-500" data-testid="runtime-profile-governance">
          {selected.governance}
          {selected.id === "mock_adapter" && (
            <span className="mt-1 block text-cyan-400/90">{MOCK_ADAPTER_GOVERNANCE}</span>
          )}
          {selected.id === "live" && (
            <span className="mt-1 block text-emerald-400/90">{LIVE_GAZEBO_GOVERNANCE}</span>
          )}
        </p>
      )}
      <p className="text-[10px] text-slate-600">
        Default: {DEFAULT_SESSION_RUNTIME_PROFILE === "stub" ? "Stub runtime" : "stub"}.
        Live Gazebo requires local ROS 2 + Gazebo + sourced rt_sandbox_gz workspace.
      </p>
    </fieldset>
  );
}
