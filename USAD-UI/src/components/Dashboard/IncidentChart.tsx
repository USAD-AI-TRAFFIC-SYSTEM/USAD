import { useMemo } from "react";
import { BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from "recharts";

interface IncidentChartProps {
  violations: Record<string, string>[];
  accidents: Record<string, string>[];
}

export default function IncidentChart({ violations, accidents }: IncidentChartProps) {
  const data = useMemo(() => {
    const hourCounts: Record<string, { hour: string; violations: number; accidents: number }> = {};
    for (let i = 0; i < 24; i++) {
      const h = String(i).padStart(2, "0");
      hourCounts[h] = { hour: `${h}:00`, violations: 0, accidents: 0 };
    }
    for (const v of violations) {
      const ts = v.timestamp || v.time || "";
      const hour = extractHour(ts);
      if (hour !== null && hourCounts[hour]) hourCounts[hour].violations++;
    }
    for (const a of accidents) {
      const ts = a.timestamp || a.time || "";
      const hour = extractHour(ts);
      if (hour !== null && hourCounts[hour]) hourCounts[hour].accidents++;
    }
    return Object.values(hourCounts);
  }, [violations, accidents]);

  const hasData = data.some((d) => d.violations > 0 || d.accidents > 0);

  return (
    <div className="bg-white border border-gray-200 rounded-xl p-5 shadow-sm">
      <h3 className="text-sm font-bold text-gray-800 mb-4 tracking-tight">Hourly Incident Breakdown</h3>
      {hasData ? (
        <ResponsiveContainer width="100%" height={280}>
          <BarChart data={data}>
            <CartesianGrid strokeDasharray="3 3" stroke="#F3F4F6" />
            <XAxis dataKey="hour" tick={{ fill: "#9CA3AF", fontSize: 10 }} axisLine={{ stroke: "#E5E7EB" }} />
            <YAxis tick={{ fill: "#9CA3AF", fontSize: 10 }} axisLine={{ stroke: "#E5E7EB" }} allowDecimals={false} />
            <Tooltip contentStyle={{ backgroundColor: "#fff", border: "1px solid #E5E7EB", borderRadius: "10px", color: "#111827", fontSize: "12px", boxShadow: "0 4px 12px rgba(0,0,0,0.08)" }} />
            <Legend wrapperStyle={{ fontSize: "11px", color: "#6B7280" }} />
            <Bar dataKey="violations" name="Violations" fill="#F97316" radius={[4, 4, 0, 0]} />
            <Bar dataKey="accidents"  name="Accidents"  fill="#EF4444" radius={[4, 4, 0, 0]} />
          </BarChart>
        </ResponsiveContainer>
      ) : (
        <div className="h-[280px] flex items-center justify-center text-gray-400 text-sm">No incident data recorded yet</div>
      )}
    </div>
  );
}

function extractHour(timestamp: string): string | null {
  const num = Number(timestamp);
  if (!isNaN(num) && num > 1000000000) return String(new Date(num * 1000).getHours()).padStart(2, "0");
  const match = timestamp.match(/(\d{2}):\d{2}/);
  if (match) return match[1];
  return null;
}
