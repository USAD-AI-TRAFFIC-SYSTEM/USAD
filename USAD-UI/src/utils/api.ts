const API_BASE = "";

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

export async function fetchLogs(
  type: "violations" | "accidents" | "traffic"
): Promise<Record<string, string>[]> {
  const res = await fetch(`${API_BASE}/api/logs/${type}`);
  return res.json();
}

export async function fetchSummary() {
  const res = await fetch(`${API_BASE}/api/logs/summary`);
  return res.json();
}

export function getVideoFeedUrl(): string {
  return `${API_BASE}/api/video/feed`;
}

export function getTelemetryWsUrl(): string {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${window.location.host}/ws/telemetry`;
}
