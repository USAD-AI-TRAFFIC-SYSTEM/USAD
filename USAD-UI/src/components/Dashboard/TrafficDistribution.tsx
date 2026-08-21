import { useMemo } from "react";
import { PieChart, Pie, Cell, ResponsiveContainer, Tooltip, Legend } from "recharts";
import { LANE_NAMES } from "../../types/telemetry";

interface TrafficDistributionProps {
  violations: Record<string, string>[];
}

const LANE_COLORS: Record<string, string> = {
  LANE1: "#F97316", LANE2: "#0891B2", LANE3: "#8B5CF6", LANE4: "#10B981",
  UNKNOWN: "#9CA3AF", INTERSECTION: "#6B7280",
};

export default function TrafficDistribution({ violations }: TrafficDistributionProps) {
  const data = useMemo(() => {
    const counts: Record<string, number> = {};
    for (const v of violations) {
      const lane = (v.lane || v.lane_key || "UNKNOWN").toUpperCase();
      counts[lane] = (counts[lane] || 0) + 1;
    }
    return Object.entries(counts).map(([name, value]) => ({ name: LANE_NAMES[name] || name, value, key: name }));
  }, [violations]);

  if (data.length === 0) {
    return (
      <div className="bg-white border border-gray-200 rounded-xl p-5 shadow-sm">
        <h3 className="text-sm font-bold text-gray-800 mb-4">Violation Distribution by Lane</h3>
        <div className="h-[240px] flex items-center justify-center text-gray-400 text-sm">No violation data recorded yet</div>
      </div>
    );
  }

  return (
    <div className="bg-white border border-gray-200 rounded-xl p-5 shadow-sm">
      <h3 className="text-sm font-bold text-gray-800 mb-4">Violation Distribution by Lane</h3>
      <ResponsiveContainer width="100%" height={240}>
        <PieChart>
          <Pie data={data} cx="50%" cy="50%" innerRadius={55} outerRadius={85} paddingAngle={3} dataKey="value">
            {data.map((entry) => <Cell key={entry.key} fill={LANE_COLORS[entry.key] ?? "#9CA3AF"} />)}
          </Pie>
          <Tooltip contentStyle={{ backgroundColor: "#fff", border: "1px solid #E5E7EB", borderRadius: "10px", fontSize: "12px", boxShadow: "0 4px 12px rgba(0,0,0,0.08)" }} />
          <Legend wrapperStyle={{ fontSize: "11px", color: "#6B7280" }} />
        </PieChart>
      </ResponsiveContainer>
    </div>
  );
}
