import { Car, AlertTriangle, ShieldAlert, Phone, ScanLine } from "lucide-react";
import type { TelemetryData } from "../../types/telemetry";
import { LANE_NAMES, LANE_KEYS } from "../../types/telemetry";
import StoplightItem from "./StoplightItem";

interface FloatingHUDProps {
  telemetry: TelemetryData | null;
}

export default function FloatingHUD({ telemetry }: FloatingHUDProps) {
  const t = telemetry;

  return (
    <div className="absolute top-4 right-4 w-[300px] flex flex-col gap-3 z-10">

      {/* ── USAD Brand Header ────────────────────────────────── */}
      <div className="glass-card px-4 py-3">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2.5">
            {/* White logo badge — real logo image */}
            <div className="w-8 h-8 rounded-lg bg-white border border-gray-200 shadow-sm flex items-center justify-center overflow-hidden flex-shrink-0">
              <img
                src="/logo.ico"
                alt="USAD Logo"
                className="w-full h-full object-contain p-0.5"
              />
            </div>
            <div className="flex flex-col leading-none">
              <span className="text-sm font-extrabold text-orange-500 tracking-tight">USAD</span>
              <span className="text-[9px] text-gray-400 tracking-wide">Smarter Signals. Safer Streets.</span>
            </div>
          </div>
          {/* Arduino + FPS */}
          <div className="text-right">
            <div className="flex items-center justify-end gap-1.5 text-[10px]">
              <span className={`w-1.5 h-1.5 rounded-full ${t?.arduino_connected ? "bg-emerald-500 animate-pulse" : "bg-amber-400"}`} />
              <span className={`font-semibold ${t?.arduino_connected ? "text-emerald-600" : "text-amber-600"}`}>
                {t?.arduino_connected ? "Connected" : "Simulation"}
              </span>
            </div>
            <span className="text-[10px] font-mono text-gray-400">
              {t?.fps?.toFixed(1) ?? "—"} FPS
            </span>
          </div>
        </div>
      </div>

      {/* ── Active Signal Card ──────────────────────────────── */}
      <div className="glass-card px-4 py-3">
        <div className="flex items-center justify-between mb-2">
          <h3 className="text-[10px] font-bold text-gray-500 uppercase tracking-widest">
            Active Signal
          </h3>
          <span className={`text-[9px] font-bold px-2 py-0.5 rounded-full ${
            t?.software_auto_mode
              ? "bg-orange-100 text-orange-600"
              : "bg-teal-100 text-teal-700"
          }`}>
            {t?.software_auto_mode ? "AUTO" : "MANUAL"}
          </span>
        </div>

        {t?.current_active_lane ? (
          <div className="flex items-center gap-3">
            <div className={`w-11 h-11 rounded-xl flex items-center justify-center text-lg font-black border-2 ${
              t.current_phase === "GREEN"
                ? "border-emerald-200 bg-emerald-50 text-emerald-600"
                : t.current_phase === "YELLOW"
                ? "border-amber-200 bg-amber-50 text-amber-600"
                : "border-red-200 bg-red-50 text-red-600"
            }`}>
              {LANE_NAMES[t.current_active_lane]?.[0] ?? "?"}
            </div>
            <div className="flex-1">
              <p className="text-sm font-bold text-gray-800">
                {LANE_NAMES[t.current_active_lane] ?? t.current_active_lane}
              </p>
              <p className={`text-xs font-semibold ${
                t.current_phase === "GREEN" ? "text-emerald-600"
                : t.current_phase === "YELLOW" ? "text-amber-600"
                : "text-red-600"
              }`}>{t.current_phase}</p>
            </div>
            <div className="text-right">
              <p className="text-2xl font-black font-mono text-orange-500 leading-none">
                {t.phase_time_remaining.toFixed(1)}
              </p>
              <p className="text-[9px] text-gray-400 mt-0.5">seconds left</p>
            </div>
          </div>
        ) : (
          <p className="text-sm text-gray-400 py-1">No active lane</p>
        )}
      </div>

      {/* ── Lane Status ─────────────────────────────────────── */}
      <div className="glass-card px-3 py-3">
        <h3 className="text-[10px] font-bold text-gray-500 uppercase tracking-widest mb-2 px-1">
          Lane Status
        </h3>
        <div className="flex flex-col gap-0.5">
          {LANE_KEYS.map((lk) => (
            <StoplightItem
              key={lk}
              laneKey={lk}
              laneName={LANE_NAMES[lk] ?? lk}
              signal={t?.lane_signals?.[lk] ?? "RED"}
              isActive={t?.current_active_lane === lk}
              timeRemaining={t?.current_active_lane === lk ? t.phase_time_remaining : undefined}
              vehicleCount={t?.lane_counts?.[lk] ?? 0}
              congestionState={t?.lane_states?.[lk] ?? "EMPTY"}
            />
          ))}
        </div>
      </div>

      {/* ── Stats Grid ──────────────────────────────────────── */}
      <div className="glass-card px-3 py-3">
        <div className="grid grid-cols-3 gap-2">
          <MiniStat icon={<Car className="w-4 h-4" />}         label="Vehicles"   value={t?.total_vehicles ?? 0}              color="orange" />
          <MiniStat icon={<ShieldAlert className="w-4 h-4" />} label="Violations" value={t?.total_violations_session ?? 0}    color={(t?.total_violations_session ?? 0) > 0 ? "amber" : "gray"} />
          <MiniStat icon={<AlertTriangle className="w-4 h-4"/>} label="Accidents"  value={t?.active_accidents_count ?? 0}      color={(t?.active_accidents_count ?? 0) > 0 ? "red" : "gray"} />
        </div>

        {(t?.emergency_notifications_count ?? 0) > 0 && (
          <div className="mt-2 pt-2 border-t border-gray-100">
            <div className="flex items-center gap-2 px-1">
              <Phone className="w-3.5 h-3.5 text-red-500 animate-pulse" />
              <span className="text-xs text-red-600 font-semibold">
                {t?.emergency_notifications_count} Emergency Alert{(t?.emergency_notifications_count ?? 0) !== 1 ? "s" : ""} Sent
              </span>
            </div>
          </div>
        )}

        {t?.detected_license_plates && t.detected_license_plates.length > 0 && (
          <div className="mt-2 pt-2 border-t border-gray-100">
            <div className="flex items-center gap-1.5 text-[10px] text-teal-600 mb-1 px-1">
              <ScanLine className="w-3 h-3" />
              <span className="font-semibold">Detected Plates</span>
            </div>
            <div className="flex flex-wrap gap-1 px-1">
              {t.detected_license_plates.map((p, i) => (
                <span key={i} className="text-[10px] font-mono bg-teal-50 text-teal-700 border border-teal-200 px-1.5 py-0.5 rounded">
                  {p.text} <span className="text-teal-400">{(p.confidence * 100).toFixed(0)}%</span>
                </span>
              ))}
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

/* ── MiniStat ───────────────────────────────────────────────── */
function MiniStat({ icon, label, value, color }: {
  icon: React.ReactNode; label: string; value: number;
  color: "orange" | "amber" | "red" | "gray";
}) {
  const map: Record<string, { bg: string; icon: string; val: string }> = {
    orange: { bg: "bg-orange-50",  icon: "text-orange-500", val: "text-orange-600" },
    amber:  { bg: "bg-amber-50",   icon: "text-amber-500",  val: "text-amber-600"  },
    red:    { bg: "bg-red-50",     icon: "text-red-500",    val: "text-red-600"    },
    gray:   { bg: "bg-gray-50",    icon: "text-gray-400",   val: "text-gray-500"   },
  };
  const c = map[color] ?? map.gray;
  return (
    <div className={`flex flex-col items-center py-2.5 px-1 rounded-xl ${c.bg}`}>
      <div className={`${c.icon} mb-1`}>{icon}</div>
      <p className={`text-xl font-black leading-none ${c.val}`}>{value}</p>
      <p className="text-[9px] text-gray-400 mt-1 text-center font-medium">{label}</p>
    </div>
  );
}
