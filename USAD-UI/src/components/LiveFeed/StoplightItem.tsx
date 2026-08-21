interface StoplightItemProps {
  laneName: string;
  laneKey: string;
  signal: "GREEN" | "YELLOW" | "RED";
  isActive: boolean;
  timeRemaining?: number;
  vehicleCount: number;
  congestionState: string;
}

const CONGESTION_BADGES: Record<string, { text: string; className: string }> = {
  CONGESTED:       { text: "Congested",     className: "text-red-600 bg-red-50 border border-red-100" },
  "NON-CONGESTED": { text: "Not Congested", className: "text-amber-600 bg-amber-50 border border-amber-100" },
  EMPTY:           { text: "Empty",         className: "text-gray-400 bg-gray-50 border border-gray-100" },
};

export default function StoplightItem({
  laneName, signal, isActive, timeRemaining, vehicleCount, congestionState,
}: StoplightItemProps) {
  const badge = CONGESTION_BADGES[congestionState] ?? CONGESTION_BADGES.EMPTY;

  return (
    <div className={`flex items-center justify-between px-2 py-2 rounded-lg transition-all duration-300 ${
      isActive ? "bg-orange-50 ring-1 ring-orange-200" : "hover:bg-gray-50"
    }`}>
      {/* Lane name + LEDs */}
      <div className="flex items-center gap-2">
        <span className={`text-[11px] font-bold w-11 ${isActive ? "text-orange-600" : "text-gray-600"}`}>
          {laneName}
        </span>
        <div className="flex items-center gap-1.5">
          {(["RED", "YELLOW", "GREEN"] as const).map((c) => (
            <div key={c} className={`w-4 h-4 rounded-full transition-all duration-500 ${
              signal === c
                ? c === "RED"   ? "led-active-red"
                : c === "YELLOW"? "led-active-amber"
                :                 "led-active-green"
                : "led-inactive"
            }`} />
          ))}
        </div>
        {isActive && timeRemaining !== undefined && (
          <span className="text-[11px] font-mono font-bold text-orange-500 min-w-[36px]">
            {timeRemaining.toFixed(1)}s
          </span>
        )}
      </div>

      {/* Count + badge */}
      <div className="flex items-center gap-1.5">
        <span className="text-xs font-bold text-gray-600 min-w-[16px] text-right font-mono">
          {vehicleCount}
        </span>
        <span className={`text-[9px] font-semibold px-1.5 py-0.5 rounded-full ${badge.className}`}>
          {badge.text}
        </span>
      </div>
    </div>
  );
}
