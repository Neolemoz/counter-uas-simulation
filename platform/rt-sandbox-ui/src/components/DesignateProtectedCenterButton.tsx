import { Shield } from "lucide-react";
import { isDesignatedProtectedCenter } from "@/cesium/defenseZoneVisualState";

const DEFAULT_BTN_CLASS =
  "inline-flex items-center gap-1.5 rounded border border-emerald-600 bg-emerald-900/90 px-3 py-2 text-xs font-semibold text-emerald-50 shadow-sm hover:bg-emerald-800 disabled:cursor-not-allowed disabled:opacity-40";

export function DesignateProtectedCenterButton({
  selectedEntityId,
  protectedCenterEntityId = null,
  disabled = false,
  onDesignate,
  className = DEFAULT_BTN_CLASS,
  dataTestId = "designate-protected-center",
}: {
  selectedEntityId: string | null;
  protectedCenterEntityId?: string | null;
  disabled?: boolean;
  onDesignate: () => void;
  className?: string;
  dataTestId?: string;
}) {
  const isDesignated =
    selectedEntityId != null &&
    isDesignatedProtectedCenter(selectedEntityId, protectedCenterEntityId);

  return (
    <button
      type="button"
      data-testid={dataTestId}
      onClick={onDesignate}
      disabled={disabled || !selectedEntityId || isDesignated}
      className={className}
      title={
        isDesignated
          ? "This entity is the designated protected center"
          : "Designate selected entity as protected center for threat evaluation"
      }
    >
      <Shield className="h-4 w-4" />
      {isDesignated ? "Protected center" : "Designate Protected Center"}
    </button>
  );
}
