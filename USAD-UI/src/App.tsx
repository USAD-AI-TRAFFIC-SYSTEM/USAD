import { useState, useEffect } from "react";
import Navbar from "./components/Header/Navbar";
import VideoCanvas from "./components/LiveFeed/VideoCanvas";
import FloatingHUD from "./components/LiveFeed/FloatingHUD";
import ActionDock from "./components/LiveFeed/ActionDock";
import AlertToast from "./components/LiveFeed/AlertToast";
import StatCard from "./components/Dashboard/StatCard";
import IncidentChart from "./components/Dashboard/IncidentChart";
import TrafficDistribution from "./components/Dashboard/TrafficDistribution";
import VehicleTypeDistribution from "./components/Dashboard/VehicleTypeDistribution";
import AccidentDurationChart from "./components/Dashboard/AccidentDurationChart";
import CategoryBreakdownChart from "./components/Dashboard/CategoryBreakdownChart";
import LogTable from "./components/Dashboard/LogTable";
import LaneCalibrator from "./components/Calibration/LaneCalibrator";
import { useTelemetry } from "./hooks/useTelemetry";
import { useKeyboardShortcuts } from "./hooks/useKeyboardShortcuts";
import { fetchLogs, fetchSummary } from "./utils/api";
import {
  ShieldAlert,
  AlertTriangle,
  Phone,
  Activity,
} from "lucide-react";

export default function App() {
  const [activeTab, setActiveTab] = useState("live");
  const { telemetry, connected } = useTelemetry();

  useKeyboardShortcuts();

  return (
    <div className="h-screen w-screen flex flex-col bg-gray-100 text-gray-900 overflow-hidden">
      <Navbar
        telemetry={telemetry}
        wsConnected={connected}
        activeTab={activeTab}
        onTabChange={setActiveTab}
      />
      <main className="flex-1 overflow-hidden">
        {activeTab === "live" ? (
          <LiveFeedView telemetry={telemetry} />
        ) : activeTab === "calibration" ? (
          <LaneCalibrator />
        ) : (
          <AnalyticsView />
        )}
      </main>
    </div>
  );
}

/* ── Live Feed View ────────────────────────────────────────── */

function LiveFeedView({
  telemetry,
}: {
  telemetry: ReturnType<typeof useTelemetry>["telemetry"];
}) {
  return (
    <div className="relative w-full h-full">
      <VideoCanvas />
      <FloatingHUD telemetry={telemetry} />
      <ActionDock />
      <AlertToast telemetry={telemetry} />
    </div>
  );
}

/* ── Analytics View ────────────────────────────────────────── */

function AnalyticsView() {
  const [violations, setViolations] = useState<Record<string, string>[]>([]);
  const [accidents, setAccidents] = useState<Record<string, string>[]>([]);
  const [traffic, setTraffic] = useState<Record<string, string>[]>([]);
  const [plates, setPlates] = useState<Record<string, string>[]>([]);
  const [summary, setSummary] = useState<Record<string, unknown> | null>(null);
  const [logTab, setLogTab] = useState<"violations" | "accidents" | "traffic" | "plates">(
    "violations"
  );

  useEffect(() => {
    const load = () => {
      fetchLogs("violations").then(setViolations).catch(() => {});
      fetchLogs("accidents").then(setAccidents).catch(() => {});
      fetchLogs("traffic").then(setTraffic).catch(() => {});
      fetchLogs("plates").then(setPlates).catch(() => {});
      fetchSummary().then(setSummary).catch(() => {});
    };
    load();
    const interval = setInterval(load, 10000);
    return () => clearInterval(interval);
  }, []);

  const vTotal = (summary as Record<string, Record<string, number>>)?.violations?.total ?? violations.length;
  const aTotal = (summary as Record<string, Record<string, number>>)?.accidents?.total ?? accidents.length;
  const eTotal = (summary as Record<string, number>)?.emergency_notifications ?? 
    accidents.filter(r => (r.emergency_notified || "").toLowerCase() === "yes").length;

  return (
    <div className="h-full overflow-y-auto p-6 space-y-6 bg-gray-100">
      {/* ── 1. KPI Overview Cards ───────────────────────────────── */}
      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
        <StatCard
          icon={<ShieldAlert className="w-5 h-5" />}
          label="Total Violations"
          value={vTotal}
          subtitle="Red light infractions"
          color="amber"
        />
        <StatCard
          icon={<AlertTriangle className="w-5 h-5" />}
          label="Total Accidents"
          value={aTotal}
          subtitle="All collisions"
          color="red"
        />
        <StatCard
          icon={<Phone className="w-5 h-5" />}
          label="Emergency Alerts"
          value={eTotal}
          subtitle="Notifications sent"
          color="purple"
        />
        <StatCard
          icon={<Activity className="w-5 h-5" />}
          label="Traffic Events"
          value={traffic.length}
          subtitle="Combined system log"
          color="teal"
        />
      </div>

      {/* ── 2. Primary Charts (Hourly Breakdown & Lane Distribution) ── */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
        <div className="lg:col-span-2">
          <IncidentChart violations={violations} accidents={accidents} />
        </div>
        <TrafficDistribution violations={violations} />
      </div>

      {/* ── 3. Vehicle Types Breakdown ──────────────────────────── */}
      <VehicleTypeDistribution traffic={traffic} />

      {/* ── 4. Secondary Breakdown Charts ───────────────────────── */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <AccidentDurationChart accidents={accidents} />
        <CategoryBreakdownChart
          title="Violation Types"
          records={violations}
          field="violation_type"
          color="#f59e0b"
        />
        <CategoryBreakdownChart
          title="Accident Types"
          records={accidents}
          field="accident_type"
          color="#ef4444"
        />
      </div>

      {/* ── 5. Detailed Filterable Data Tables ──────────────────── */}
      <div>
        <div className="flex items-center justify-between mb-3">
          <h3 className="text-sm font-semibold text-gray-900">Log Records Explorer</h3>
          <div className="flex items-center gap-1">
            {(
              [
                { id: "violations", label: `Violations (${violations.length})` },
                { id: "accidents", label: `Accidents (${accidents.length})` },
                { id: "traffic", label: `Traffic Events (${traffic.length})` },
                { id: "plates", label: `Nameplates (${plates.length})` },
              ] as const
            ).map((tab) => (
              <button
                key={tab.id}
                onClick={() => setLogTab(tab.id)}
                className={`px-3 py-1.5 rounded-lg text-xs font-medium transition-all ${
                  logTab === tab.id
                    ? "bg-white text-orange-600 shadow-sm ring-1 ring-orange-200"
                    : "text-gray-500 hover:text-gray-700 hover:bg-white/60"
                }`}
              >
                {tab.label}
              </button>
            ))}
          </div>
        </div>

        {logTab === "violations" && (
          <LogTable title="Violation Logs" records={violations} />
        )}
        {logTab === "accidents" && (
          <LogTable title="Accident Logs" records={accidents} />
        )}
        {logTab === "traffic" && (
          <LogTable title="Traffic Event Logs" records={traffic} />
        )}
        {logTab === "plates" && (
          <LogTable title="License Plate Logs" records={plates} />
        )}
      </div>
    </div>
  );
}
