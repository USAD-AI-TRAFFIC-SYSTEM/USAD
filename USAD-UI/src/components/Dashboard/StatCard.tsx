interface StatCardProps {
  icon: React.ReactNode;
  label: string;
  value: number | string;
  subtitle?: string;
  color: "orange" | "red" | "amber" | "emerald" | "purple" | "teal" | "slate";
}

const COLOR_MAP: Record<string, { icon: string; value: string; border: string }> = {
  orange:  { icon: "text-orange-500 bg-orange-50",  value: "text-orange-600", border: "border-orange-100" },
  red:     { icon: "text-red-500 bg-red-50",         value: "text-red-600",    border: "border-red-100"   },
  amber:   { icon: "text-amber-500 bg-amber-50",     value: "text-amber-600",  border: "border-amber-100" },
  emerald: { icon: "text-emerald-500 bg-emerald-50", value: "text-emerald-600",border: "border-emerald-100"},
  purple:  { icon: "text-purple-500 bg-purple-50",   value: "text-purple-600", border: "border-purple-100"},
  teal:    { icon: "text-teal-500 bg-teal-50",       value: "text-teal-600",   border: "border-teal-100"  },
  slate:   { icon: "text-gray-500 bg-gray-50",       value: "text-gray-600",   border: "border-gray-100"  },
};

export default function StatCard({ icon, label, value, subtitle, color }: StatCardProps) {
  const c = COLOR_MAP[color] ?? COLOR_MAP.slate;
  return (
    <div className={`bg-white border ${c.border} rounded-xl p-5 shadow-sm hover:shadow-md transition-shadow duration-200`}>
      <div className="flex items-start justify-between">
        <div>
          <p className="text-[10px] font-bold text-gray-400 uppercase tracking-widest mb-1.5">{label}</p>
          <p className={`text-3xl font-black leading-none ${c.value}`}>{value}</p>
          {subtitle && <p className="text-[11px] text-gray-400 mt-1.5">{subtitle}</p>}
        </div>
        <div className={`p-2.5 rounded-xl ${c.icon}`}>{icon}</div>
      </div>
    </div>
  );
}
