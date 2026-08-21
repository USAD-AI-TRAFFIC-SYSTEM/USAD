import { AlertTriangle, ShieldAlert, X } from "lucide-react";
import { useState, useEffect } from "react";
import type { TelemetryData } from "../../types/telemetry";

interface AlertToastProps {
  telemetry: TelemetryData | null;
}

interface Toast {
  id: number;
  type: "accident" | "violation";
  message: string;
  timestamp: number;
}

let toastId = 0;

export default function AlertToast({ telemetry }: AlertToastProps) {
  const [toasts, setToasts] = useState<Toast[]>([]);
  const [prevAccidents, setPrevAccidents] = useState(0);
  const [prevViolations, setPrevViolations] = useState(0);

  useEffect(() => {
    if (!telemetry) return;
    const newToasts: Toast[] = [];
    if (telemetry.active_accidents_count > prevAccidents) {
      newToasts.push({ id: ++toastId, type: "accident", message: `Accident detected! (${telemetry.active_accidents_count} active)`, timestamp: Date.now() });
    }
    if (telemetry.total_violations_session > prevViolations) {
      newToasts.push({ id: ++toastId, type: "violation", message: `Red-light violation detected! (${telemetry.total_violations_session} total)`, timestamp: Date.now() });
    }
    setPrevAccidents(telemetry.active_accidents_count);
    setPrevViolations(telemetry.total_violations_session);
    if (newToasts.length > 0) setToasts((prev) => [...prev, ...newToasts].slice(-5));
  }, [telemetry?.active_accidents_count, telemetry?.total_violations_session]);

  useEffect(() => {
    const interval = setInterval(() => {
      const now = Date.now();
      setToasts((prev) => prev.filter((t) => now - t.timestamp < 5000));
    }, 1000);
    return () => clearInterval(interval);
  }, []);

  if (toasts.length === 0) return null;

  return (
    <div className="absolute top-4 left-4 z-20 flex flex-col gap-2 max-w-sm">
      {toasts.map((toast) => (
        <div key={toast.id}
          className={`flex items-center gap-3 px-4 py-3 rounded-xl border shadow-lg animate-slide-in ${
            toast.type === "accident"
              ? "bg-red-50 border-red-200 text-red-700"
              : "bg-amber-50 border-amber-200 text-amber-700"
          }`}
        >
          {toast.type === "accident"
            ? <AlertTriangle className="w-5 h-5 flex-shrink-0" />
            : <ShieldAlert className="w-5 h-5 flex-shrink-0" />}
          <p className="text-sm font-semibold flex-1">{toast.message}</p>
          <button onClick={() => setToasts((p) => p.filter((t) => t.id !== toast.id))}
            className="p-0.5 rounded hover:bg-black/5 transition-colors">
            <X className="w-3.5 h-3.5" />
          </button>
        </div>
      ))}
    </div>
  );
}
