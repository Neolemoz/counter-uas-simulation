function finiteNumber(value: number | null | undefined): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

export function formatNumber(value: number | null | undefined, digits = 1): string {
  const parsed = finiteNumber(value);
  return parsed === null ? "-" : parsed.toFixed(digits);
}

export function formatMeters(value: number | null | undefined): string {
  const parsed = finiteNumber(value);
  return parsed === null ? "-" : `${parsed.toFixed(1)} m`;
}

export function formatMps(value: number | null | undefined): string {
  const parsed = finiteNumber(value);
  return parsed === null ? "-" : `${parsed.toFixed(2)} m/s`;
}

export function formatHeading(value: number | null | undefined): string {
  const parsed = finiteNumber(value);
  return parsed === null ? "-" : `${parsed.toFixed(1)} deg`;
}

export function formatTimestamp(value: string | null | undefined): string {
  return value && value.trim().length > 0 ? value : "-";
}

export function labelFromToken(value: string): string {
  const label = value
    .split("_")
    .filter(Boolean)
    .join(" ");
  return label.charAt(0).toUpperCase() + label.slice(1);
}
