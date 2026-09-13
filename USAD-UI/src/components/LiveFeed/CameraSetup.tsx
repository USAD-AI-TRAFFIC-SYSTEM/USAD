import { useEffect, useState } from "react";
import { Camera, RefreshCw, Save, ScanLine, X } from "lucide-react";
import {
  fetchCameraConfiguration,
  saveCameraConfiguration,
  type CameraConfiguration,
  type CameraDevice,
} from "../../utils/api";

interface CameraSetupProps {
  onClose: () => void;
}

function deviceDescription(device: CameraDevice) {
  const mode = device.width && device.height ? ` - ${device.width}x${device.height}` : "";
  return `${device.label}${mode}${device.active ? " (active)" : ""}`;
}

export default function CameraSetup({ onClose }: CameraSetupProps) {
  const [configuration, setConfiguration] = useState<CameraConfiguration | null>(null);
  const [birdEye, setBirdEye] = useState<number | null>(null);
  const [plates, setPlates] = useState<number | null>(null);
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState("");

  const load = async () => {
    setLoading(true);
    setError("");
    try {
      const data = await fetchCameraConfiguration(true);
      setConfiguration(data);
      setBirdEye(data.assignments.vehicle_detection);
      setPlates(data.assignments.license_plate);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Camera scan failed.");
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    let cancelled = false;
    fetchCameraConfiguration(true)
      .then((data) => {
        if (cancelled) return;
        setConfiguration(data);
        setBirdEye(data.assignments.vehicle_detection);
        setPlates(data.assignments.license_plate);
      })
      .catch((err) => {
        if (!cancelled) setError(err instanceof Error ? err.message : "Camera scan failed.");
      })
      .finally(() => {
        if (!cancelled) setLoading(false);
      });
    return () => {
      cancelled = true;
    };
  }, []);

  const save = async () => {
    if (birdEye === null || plates === null) return;
    if (birdEye === plates) {
      setError("Choose a different physical camera for each view.");
      return;
    }
    setSaving(true);
    setError("");
    try {
      await saveCameraConfiguration({ vehicle_detection: birdEye, license_plate: plates });
      onClose();
    } catch (err) {
      setError(err instanceof Error ? err.message : "Unable to save camera assignments.");
    } finally {
      setSaving(false);
    }
  };

  const devices = configuration?.devices ?? [];
  const selectable = devices.filter((device) => device.available);

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-gray-950/55 p-4" onMouseDown={onClose}>
      <div
        className="w-full max-w-lg rounded-2xl bg-white shadow-2xl ring-1 ring-black/10"
        onMouseDown={(event) => event.stopPropagation()}
      >
        <div className="flex items-start justify-between border-b border-gray-200 px-5 py-4">
          <div>
            <h2 className="flex items-center gap-2 text-base font-bold text-gray-900">
              <Camera className="h-4 w-4 text-teal-600" /> Camera assignments
            </h2>
            <p className="mt-1 text-xs text-gray-500">
              Assign a physical input to each USAD view. Press C to switch between them.
            </p>
          </div>
          <button onClick={onClose} className="rounded-lg p-1.5 text-gray-400 hover:bg-gray-100 hover:text-gray-700">
            <X className="h-4 w-4" />
          </button>
        </div>

        <div className="space-y-4 px-5 py-5">
          {loading ? (
            <div className="flex items-center justify-center gap-2 py-10 text-sm text-gray-500">
              <RefreshCw className="h-4 w-4 animate-spin" /> Detecting connected cameras...
            </div>
          ) : (
            <>
              <CameraSelect
                icon={<Camera className="h-4 w-4" />}
                title="Bird's-eye traffic view"
                description="North, East, South and West vehicle detection"
                devices={devices}
                availableDevices={selectable}
                value={birdEye}
                onChange={setBirdEye}
              />
              <CameraSelect
                icon={<ScanLine className="h-4 w-4" />}
                title="License-plate view"
                description="Lower-angle view used to read vehicle plates"
                devices={devices}
                availableDevices={selectable}
                value={plates}
                onChange={setPlates}
              />

              <div className="rounded-lg bg-gray-50 px-3 py-2 text-[11px] text-gray-500">
                Detected {selectable.length} available camera{selectable.length === 1 ? "" : "s"}.
                Device numbers are assigned by Windows and may change after reconnecting USB cameras.
              </div>
            </>
          )}

          {error && <p className="rounded-lg bg-red-50 px-3 py-2 text-xs font-medium text-red-700">{error}</p>}
        </div>

        <div className="flex items-center justify-between border-t border-gray-200 px-5 py-4">
          <button
            onClick={() => void load()}
            disabled={loading || saving}
            className="flex items-center gap-1.5 rounded-lg px-3 py-2 text-xs font-semibold text-teal-700 hover:bg-teal-50 disabled:opacity-50"
          >
            <RefreshCw className={`h-3.5 w-3.5 ${loading ? "animate-spin" : ""}`} /> Rescan
          </button>
          <div className="flex gap-2">
            <button onClick={onClose} className="rounded-lg px-3 py-2 text-xs font-semibold text-gray-600 hover:bg-gray-100">
              Cancel
            </button>
            <button
              onClick={() => void save()}
              disabled={loading || saving || birdEye === null || plates === null || birdEye === plates}
              className="flex items-center gap-1.5 rounded-lg bg-orange-500 px-3.5 py-2 text-xs font-semibold text-white hover:bg-orange-600 disabled:cursor-not-allowed disabled:opacity-50"
            >
              <Save className="h-3.5 w-3.5" /> {saving ? "Saving..." : "Save assignments"}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}

function CameraSelect({
  icon,
  title,
  description,
  devices,
  availableDevices,
  value,
  onChange,
}: {
  icon: React.ReactNode;
  title: string;
  description: string;
  devices: CameraDevice[];
  availableDevices: CameraDevice[];
  value: number | null;
  onChange: (value: number) => void;
}) {
  const selected = devices.find((device) => device.source === value);
  return (
    <label className="block rounded-xl border border-gray-200 p-3.5">
      <span className="flex items-center gap-2 text-sm font-semibold text-gray-800">
        <span className="text-teal-600">{icon}</span>{title}
      </span>
      <span className="mb-2.5 mt-0.5 block text-[11px] text-gray-500">{description}</span>
      <select
        value={value ?? ""}
        onChange={(event) => onChange(Number(event.target.value))}
        className="w-full rounded-lg border border-gray-300 bg-white px-3 py-2 text-sm outline-none focus:border-teal-500 focus:ring-2 focus:ring-teal-100"
      >
        {selected && !selected.available && (
          <option value={selected.source}>{deviceDescription(selected)}</option>
        )}
        {availableDevices.map((device) => (
          <option key={device.source} value={device.source}>{deviceDescription(device)}</option>
        ))}
      </select>
    </label>
  );
}
