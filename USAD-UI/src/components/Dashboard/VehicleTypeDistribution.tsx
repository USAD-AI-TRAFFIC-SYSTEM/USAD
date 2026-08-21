import { useMemo } from "react";

interface VehicleTypeDistributionProps {
  traffic: Record<string, string>[];
}

const TYPE_CONFIG: Record<string, { border: string; bg: string; text: string; bar: string }> = {
  SMALL:   { border: "border-orange-200", bg: "bg-orange-50",  text: "text-orange-600", bar: "bg-orange-400" },
  MEDIUM:  { border: "border-teal-200",   bg: "bg-teal-50",    text: "text-teal-600",   bar: "bg-teal-400"   },
  LARGE:   { border: "border-amber-200",  bg: "bg-amber-50",   text: "text-amber-600",  bar: "bg-amber-400"  },
  UNKNOWN: { border: "border-gray-200",   bg: "bg-gray-50",    text: "text-gray-500",   bar: "bg-gray-300"   },
};

export default function VehicleTypeDistribution({ traffic }: VehicleTypeDistributionProps) {
  const { counts, total, maxVal } = useMemo(() => {
    const c: Record<string, number> = { SMALL: 0, MEDIUM: 0, LARGE: 0, UNKNOWN: 0 };
    for (const r of traffic) {
      const vt = (r.vehicle_type || "UNKNOWN").toUpperCase();
      c[vt] = (c[vt] || 0) + 1;
    }
    return { counts: c, total: traffic.length, maxVal: Math.max(1, ...Object.values(c)) };
  }, [traffic]);

  const items = Object.entries(counts).sort((a, b) => b[1] - a[1]);

  return (
    <div className="bg-white border border-gray-200 rounded-xl p-5 shadow-sm">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-sm font-bold text-gray-800">Vehicle Types in Traffic</h3>
        <span className="text-xs text-gray-400">{total} total events</span>
      </div>
      <div className="grid grid-cols-2 sm:grid-cols-4 gap-3">
        {items.map(([type, count]) => {
          const s = TYPE_CONFIG[type] ?? TYPE_CONFIG.UNKNOWN;
          const pct = total > 0 ? ((count / total) * 100).toFixed(1) : "0.0";
          const barWidth = `${Math.min(100, (count / maxVal) * 100)}%`;
          return (
            <div key={type} className={`bg-white border ${s.border} rounded-xl p-3 shadow-sm`}>
              <div className="flex items-center justify-between mb-1">
                <span className={`text-[10px] font-bold uppercase tracking-wider ${s.text}`}>{type}</span>
                <span className="text-[10px] font-mono text-gray-400">{pct}%</span>
              </div>
              <p className={`text-xl font-black mb-2 ${s.text}`}>{count}</p>
              <div className="w-full h-1.5 bg-gray-100 rounded-full overflow-hidden">
                <div className={`h-full rounded-full ${s.bar}`} style={{ width: barWidth }} />
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}
