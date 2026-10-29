const API_BASE = "";

export type CameraRole = "vehicle_detection" | "license_plate";

export interface CameraDevice {
  source: number;
  label: string;
  available: boolean;
  active?: boolean;
  width: number | null;
  height: number | null;
  fps: number | null;
  assigned_roles?: CameraRole[];
}

export interface CameraConfiguration {
  devices: CameraDevice[];
  assignments: Record<CameraRole, number>;
  active_source: number;
  active_role: CameraRole;
}

export async function controlAction(
  action: string,
  params?: Record<string, string>
): Promise<{ ok: boolean }> {
  let url = `${API_BASE}/api/control/${action}`;
  if (params) {
    const search = new URLSearchParams(params).toString();
    if (search) url += `?${search}`;
  }
  const res = await fetch(url, { method: "POST" });
  return res.json();
}

export type LogType = "violations" | "accidents" | "traffic" | "plates";

export function getLogExportUrl(type: LogType, search = ""): string {
  const q = search.trim();
  return `${API_BASE}/api/logs/export/${type}${q ? `?q=${encodeURIComponent(q)}` : ""}`;
}

export function getAnalyticsExportUrl(): string {
  return `${API_BASE}/api/analytics/export`;
}

/** Start a download of a server-generated attachment (works in the desktop window too). */
export function downloadFile(url: string) {
  const a = Object.assign(document.createElement("a"), { href: url });
  document.body.appendChild(a);
  a.click();
  a.remove();
}

export async function fetchLogs(
  type: LogType
): Promise<Record<string, string>[]> {
  const res = await fetch(`${API_BASE}/api/logs/${type}`, { cache: "no-store" });
  if (!res.ok) throw new Error(`Unable to fetch ${type} logs (${res.status}).`);
  return res.json();
}

export async function fetchSummary() {
  const res = await fetch(`${API_BASE}/api/logs/summary`, { cache: "no-store" });
  if (!res.ok) throw new Error(`Unable to fetch analytics summary (${res.status}).`);
  return res.json();
}

export async function fetchCameraConfiguration(scan = true): Promise<CameraConfiguration> {
  const res = await fetch(`${API_BASE}/api/config/cameras?scan=${scan}`);
  if (!res.ok) throw new Error("Unable to read camera configuration.");
  return res.json();
}

export async function saveCameraConfiguration(
  assignments: Record<CameraRole, number>
): Promise<CameraConfiguration & { ok: boolean; error?: string }> {
  const res = await fetch(`${API_BASE}/api/config/cameras`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(assignments),
  });
  const body = await res.json();
  if (!res.ok) throw new Error(body.error || "Unable to save camera assignments.");
  return body;
}

export function getVideoFeedUrl(): string {
  return `${API_BASE}/api/video/feed`;
}

export function getTelemetryWsUrl(): string {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${window.location.host}/ws/telemetry`;
}
