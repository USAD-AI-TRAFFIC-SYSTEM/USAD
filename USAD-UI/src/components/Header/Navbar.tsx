import { Camera, Cpu, Activity, Clock, Power, BarChart2, Crosshair } from "lucide-react";
import type { TelemetryData } from "../../types/telemetry";

interface NavbarProps {
  telemetry: TelemetryData | null;
  wsConnected: boolean;
  activeTab: string;
  onTabChange: (tab: string) => void;
}

const TABS = [
  { id: "live",        label: "Live Camera",   icon: Camera },
  { id: "analytics",   label: "Analytics",     icon: BarChart2 },
  { id: "calibration", label: "Calibration",   icon: Crosshair },
];

export default function Navbar({ telemetry, wsConnected, activeTab, onTabChange }: NavbarProps) {
  const t = telemetry;

  return (
    <header className="flex items-center justify-between h-14 px-4 bg-white border-b border-gray-200 select-none shadow-sm">
      {/* ── Left: Logo + Brand ──────────────────────────────── */}
      <div className="flex items-center gap-5">
        {/* Logo badge — white background with orange icon */}
        <div className="flex items-center gap-2.5">
          <div className="w-9 h-9 rounded-xl bg-white border border-gray-200 shadow-sm flex items-center justify-center flex-shrink-0 overflow-hidden">
            <img
              src="/logo.ico"
              alt="USAD Logo"
              className="w-full h-full object-contain p-0.5"
            />
          </div>
          <div className="flex flex-col leading-none">
            <span className="text-base font-extrabold text-orange-500 tracking-tight">USAD</span>
            <span className="text-[9px] font-medium text-gray-400 tracking-wide whitespace-nowrap">
              Smarter Signals. Safer Streets.
            </span>
          </div>
        </div>

        {/* Separator */}
        <div className="w-px h-7 bg-gray-200" />

        {/* Tab Switcher */}
        <nav className="flex gap-1">
          {TABS.map(({ id, label, icon: Icon }) => (
            <button
              key={id}
              onClick={() => onTabChange(id)}
              className={`flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-semibold transition-all duration-200 ${
                activeTab === id
                  ? "bg-orange-50 text-orange-600 ring-1 ring-orange-200"
                  : "text-gray-500 hover:text-gray-700 hover:bg-gray-100"
              }`}
            >
              <Icon className="w-3.5 h-3.5" />
              {label}
            </button>
          ))}
        </nav>
      </div>

      {/* ── Right: Status Badges ─────────────────────────────── */}
      <div className="flex items-center gap-2">
        {/* Live / Offline */}
        <Pill color={wsConnected ? "green" : "red"} pulse={wsConnected}>
          {wsConnected ? "Live" : "Offline"}
        </Pill>

        {/* Camera */}
        <Pill color="teal" icon={<Camera className="w-3 h-3" />}>
          Cam {t?.camera_source ?? "—"}
        </Pill>

        {/* Arduino */}
        <Pill
          color={t?.arduino_connected ? "green" : "amber"}
          icon={<Cpu className="w-3 h-3" />}
        >
          {t?.arduino_connected ? "Connected" : "Simulation"}
        </Pill>

        {/* FPS */}
        <Pill color="orange" icon={<Activity className="w-3 h-3" />}>
          {t?.fps?.toFixed(1) ?? "—"} FPS
        </Pill>

        {/* Clock */}
        <div className="flex items-center gap-1.5 text-[11px] text-gray-400 ml-1">
          <Clock className="w-3 h-3" />
          <span className="font-mono">{new Date().toLocaleTimeString()}</span>
        </div>

        {/* Shutdown */}
        <button
          onClick={() => {
            if (confirm("Shut down the USAD system?")) {
              fetch("/api/control/shutdown", { method: "POST" });
            }
          }}
          title="Shutdown [Q]"
          className="p-1.5 rounded-lg text-gray-400 hover:text-red-500 hover:bg-red-50 transition-all duration-200 ml-1"
        >
          <Power className="w-4 h-4" />
        </button>
      </div>
    </header>
  );
}

/* ── Pill ───────────────────────────────────────────────────── */
function Pill({ children, color, icon, pulse }: {
  children: React.ReactNode;
  color: "green" | "red" | "amber" | "orange" | "teal";
  icon?: React.ReactNode;
  pulse?: boolean;
}) {
  const styles: Record<string, string> = {
    green:  "bg-emerald-50 text-emerald-700 ring-1 ring-emerald-200",
    red:    "bg-red-50 text-red-600 ring-1 ring-red-200",
    amber:  "bg-amber-50 text-amber-700 ring-1 ring-amber-200",
    orange: "bg-orange-50 text-orange-600 ring-1 ring-orange-200",
    teal:   "bg-teal-50 text-teal-700 ring-1 ring-teal-200",
  };
  const dots: Record<string, string> = {
    green: "bg-emerald-500", red: "bg-red-500", amber: "bg-amber-500",
    orange: "bg-orange-500", teal: "bg-teal-500",
  };

  return (
    <div className={`flex items-center gap-1.5 px-2 py-0.5 rounded-full text-[11px] font-semibold ${styles[color]}`}>
      {pulse ? (
        <span className="relative flex h-1.5 w-1.5">
          <span className={`animate-ping absolute inline-flex h-full w-full rounded-full ${dots[color]} opacity-75`} />
          <span className={`relative inline-flex rounded-full h-1.5 w-1.5 ${dots[color]}`} />
        </span>
      ) : icon}
      {children}
    </div>
  );
}
