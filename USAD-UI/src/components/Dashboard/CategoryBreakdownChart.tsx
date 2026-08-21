import { useMemo } from "react";
import { BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Cell } from "recharts";

interface CategoryBreakdownChartProps {
  title: string;
  records: Record<string, string>[];
  field: "violation_type" | "accident_type";
  color: string;
}

export default function CategoryBreakdownChart({ title, records, field, color }: CategoryBreakdownChartProps) {
  const data = useMemo(() => {
    const counts: Record<string, number> = {};
    for (const r of records) {
      const type = (r[field] || r.type || "UNKNOWN").replace(/_/g, " ");
      counts[type] = (counts[type] || 0) + 1;
    }
    return Object.entries(counts).map(([name, value]) => ({ name, value })).sort((a, b) => b.value - a.value);
  }, [records, field]);

  return (
    <div className="bg-white border border-gray-200 rounded-xl p-5 shadow-sm">
      <h3 className="text-sm font-bold text-gray-800 mb-4">{title}</h3>
      {data.length > 0 ? (
        <ResponsiveContainer width="100%" height={220}>
          <BarChart data={data} layout="vertical">
            <CartesianGrid strokeDasharray="3 3" stroke="#F3F4F6" />
            <XAxis type="number" tick={{ fill: "#9CA3AF", fontSize: 10 }} axisLine={{ stroke: "#E5E7EB" }} allowDecimals={false} />
            <YAxis dataKey="name" type="category" tick={{ fill: "#6B7280", fontSize: 10 }} axisLine={{ stroke: "#E5E7EB" }} width={120} />
            <Tooltip contentStyle={{ backgroundColor: "#fff", border: "1px solid #E5E7EB", borderRadius: "10px", fontSize: "12px", boxShadow: "0 4px 12px rgba(0,0,0,0.08)" }} />
            <Bar dataKey="value" name="Count" radius={[0, 4, 4, 0]}>
              {data.map((_, idx) => <Cell key={idx} fill={color} />)}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
      ) : (
        <div className="h-[220px] flex items-center justify-center text-gray-400 text-sm">No records found</div>
      )}
    </div>
  );
}
