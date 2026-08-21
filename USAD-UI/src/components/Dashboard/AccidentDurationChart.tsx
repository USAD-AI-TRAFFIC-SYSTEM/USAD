import { useMemo } from "react";
import { BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Cell } from "recharts";

interface AccidentDurationChartProps {
  accidents: Record<string, string>[];
}

const BUCKET_COLORS: Record<string, string> = {
  "0s (instant)": "#0891B2", "< 1s": "#10B981", "1–5s": "#F59E0B", "> 5s": "#EF4444",
};

export default function AccidentDurationChart({ accidents }: AccidentDurationChartProps) {
  const data = useMemo(() => {
    const buckets: Record<string, number> = { "0s (instant)": 0, "< 1s": 0, "1–5s": 0, "> 5s": 0 };
    for (const r of accidents) {
      const d = parseFloat(r.duration || "0") || 0;
      if (d === 0) buckets["0s (instant)"]++;
      else if (d < 1) buckets["< 1s"]++;
      else if (d <= 5) buckets["1–5s"]++;
      else buckets["> 5s"]++;
    }
    return Object.entries(buckets).map(([name, value]) => ({ name, value }));
  }, [accidents]);

  return (
    <div className="bg-white border border-gray-200 rounded-xl p-5 shadow-sm">
      <h3 className="text-sm font-bold text-gray-800 mb-4">Accident Duration Distribution</h3>
      {accidents.length > 0 ? (
        <ResponsiveContainer width="100%" height={220}>
          <BarChart data={data} layout="vertical">
            <CartesianGrid strokeDasharray="3 3" stroke="#F3F4F6" />
            <XAxis type="number" tick={{ fill: "#9CA3AF", fontSize: 10 }} axisLine={{ stroke: "#E5E7EB" }} allowDecimals={false} />
            <YAxis dataKey="name" type="category" tick={{ fill: "#6B7280", fontSize: 11 }} axisLine={{ stroke: "#E5E7EB" }} width={90} />
            <Tooltip contentStyle={{ backgroundColor: "#fff", border: "1px solid #E5E7EB", borderRadius: "10px", fontSize: "12px", boxShadow: "0 4px 12px rgba(0,0,0,0.08)" }} />
            <Bar dataKey="value" name="Accidents" radius={[0, 4, 4, 0]}>
              {data.map((entry) => <Cell key={entry.name} fill={BUCKET_COLORS[entry.name] ?? "#9CA3AF"} />)}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
      ) : (
        <div className="h-[220px] flex items-center justify-center text-gray-400 text-sm">No accident data yet</div>
      )}
    </div>
  );
}
