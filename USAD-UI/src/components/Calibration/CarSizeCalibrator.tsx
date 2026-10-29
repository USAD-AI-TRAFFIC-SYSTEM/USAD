import { useEffect, useRef, useState, useCallback } from "react";
import { Save, RotateCcw } from "lucide-react";
import { getVideoFeedUrl } from "../../utils/api";

/** [x, y, width, height] in camera coordinates. */
type Box = [number, number, number, number];

interface CarSizeConfig {
  length_range: [number, number];
  width_range: [number, number];
  tolerance: number;
  camera_width: number;
  camera_height: number;
}

type Which = "small" | "large";

const BOX_STYLE: Record<Which, { color: string; label: string }> = {
  small: { color: "#22c55e", label: "Smallest car" },
  large: { color: "#ef4444", label: "Largest car" },
};
const MIN_BOX = 8;

/** Length is the long side and width the short side, whatever the box orientation. */
const dims = (b: Box) => ({ length: Math.max(b[2], b[3]), width: Math.min(b[2], b[3]) });

export default function CarSizeCalibrator() {
  const containerRef = useRef<HTMLDivElement>(null);
  const svgRef = useRef<SVGSVGElement>(null);
  const [saved, setSaved] = useState<CarSizeConfig | null>(null);
  const [boxes, setBoxes] = useState<Record<Which, Box> | null>(null);
  const [tolerance, setTolerance] = useState(0.25);
  const [imgSize, setImgSize] = useState({ width: 1280, height: 720 });
  const [dragging, setDragging] = useState<{
    which: Which;
    mode: "move" | "resize";
    start: [number, number];
    box: Box;
  } | null>(null);
  const [dirty, setDirty] = useState(false);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const boxesFrom = useCallback((cfg: CarSizeConfig): Record<Which, Box> => {
    const cy = cfg.camera_height / 2;
    return {
      small: [Math.round(cfg.camera_width * 0.3), Math.round(cy - cfg.width_range[0] / 2), cfg.length_range[0], cfg.width_range[0]],
      large: [Math.round(cfg.camera_width * 0.55), Math.round(cy - cfg.width_range[1] / 2), cfg.length_range[1], cfg.width_range[1]],
    };
  }, []);

  const applyConfig = useCallback(
    (cfg: CarSizeConfig) => {
      setSaved(cfg);
      setImgSize({ width: cfg.camera_width, height: cfg.camera_height });
      setBoxes(boxesFrom(cfg));
      setTolerance(cfg.tolerance);
      setDirty(false);
      setError(null);
    },
    [boxesFrom]
  );

  useEffect(() => {
    fetch("/api/config/car-size")
      .then((r) => r.json())
      .then(applyConfig)
      .catch(() => setError("Could not load the car size settings."));
  }, [applyConfig]);

  // Re-render when the video area is first laid out or resized, so the boxes
  // are drawn at the right screen position.
  const ready = boxes !== null && saved !== null;
  const [containerSize, setContainerSize] = useState({ width: 0, height: 0 });
  useEffect(() => {
    const el = containerRef.current;
    if (!ready || !el) return;
    const observer = new ResizeObserver(([entry]) =>
      setContainerSize({ width: entry.contentRect.width, height: entry.contentRect.height })
    );
    observer.observe(el);
    return () => observer.disconnect();
  }, [ready]);

  // Video is letterboxed (object-contain); map camera pixels to screen pixels.
  const getScale = useCallback(() => {
    const { width, height } = containerSize;
    if (!width || !height) return { s: 1, offsetX: 0, offsetY: 0 };
    const s = Math.min(width / imgSize.width, height / imgSize.height);
    return { s, offsetX: (width - imgSize.width * s) / 2, offsetY: (height - imgSize.height * s) / 2 };
  }, [imgSize, containerSize]);

  const toNative = useCallback(
    (clientX: number, clientY: number): [number, number] => {
      const rect = svgRef.current!.getBoundingClientRect();
      const { s, offsetX, offsetY } = getScale();
      return [(clientX - rect.left - offsetX) / s, (clientY - rect.top - offsetY) / s];
    },
    [getScale]
  );

  const handleMouseDown = (e: React.MouseEvent, which: Which, mode: "move" | "resize") => {
    if (!boxes || !svgRef.current) return;
    e.preventDefault();
    e.stopPropagation();
    setDragging({ which, mode, start: toNative(e.clientX, e.clientY), box: [...boxes[which]] as Box });
  };

  const handleMouseMove = useCallback(
    (e: MouseEvent) => {
      if (!dragging || !boxes) return;
      const [px, py] = toNative(e.clientX, e.clientY);
      const dx = Math.round(px - dragging.start[0]);
      const dy = Math.round(py - dragging.start[1]);
      const [x, y, w, h] = dragging.box;
      const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
      const next: Box =
        dragging.mode === "move"
          ? [clamp(x + dx, 0, imgSize.width - w), clamp(y + dy, 0, imgSize.height - h), w, h]
          : [x, y, clamp(w + dx, MIN_BOX, imgSize.width - x), clamp(h + dy, MIN_BOX, imgSize.height - y)];
      setBoxes({ ...boxes, [dragging.which]: next });
      if (dragging.mode === "resize") setDirty(true);
    },
    [dragging, boxes, toNative, imgSize]
  );

  useEffect(() => {
    if (!dragging) return;
    const up = () => setDragging(null);
    window.addEventListener("mousemove", handleMouseMove);
    window.addEventListener("mouseup", up);
    return () => {
      window.removeEventListener("mousemove", handleMouseMove);
      window.removeEventListener("mouseup", up);
    };
  }, [dragging, handleMouseMove]);

  /** Set a box's long or short side from a number input, keeping its orientation. */
  const setSide = (which: Which, side: "length" | "width", value: number) => {
    if (!boxes || !Number.isFinite(value)) return;
    const [x, y, w, h] = boxes[which];
    const horizontal = w >= h;
    const v = Math.max(MIN_BOX, Math.round(value));
    const isW = (side === "length") === horizontal;
    setBoxes({ ...boxes, [which]: isW ? [x, y, v, h] : [x, y, w, v] });
    setDirty(true);
  };

  if (!boxes || !saved) {
    return (
      <div className="flex items-center justify-center h-full text-gray-500">
        {error ?? "Loading car size settings..."}
      </div>
    );
  }

  const small = dims(boxes.small);
  const large = dims(boxes.large);
  const invalid =
    small.length > large.length
      ? "Smallest car is longer than the largest car."
      : small.width > large.width
      ? "Smallest car is wider than the largest car."
      : large.width > large.length
      ? "Largest car is wider than it is long."
      : null;
  const accept = {
    lengthLo: Math.round(small.length * (1 - tolerance)),
    lengthHi: Math.round(large.length * (1 + tolerance)),
    widthLo: Math.round(small.width * (1 - tolerance)),
    widthHi: Math.round(large.width * (1 + tolerance)),
  };

  const handleSave = async () => {
    setSaving(true);
    setError(null);
    try {
      const res = await fetch("/api/config/car-size", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          length_range: [small.length, large.length],
          width_range: [small.width, large.width],
          tolerance,
        }),
      });
      const data = await res.json();
      if (data.ok) {
        setSaved(data);
        setDirty(false);
      } else {
        setError(data.error || "Failed to save.");
      }
    } catch {
      setError("Network error saving car size.");
    }
    setSaving(false);
  };

  const { s, offsetX, offsetY } = getScale();
  const toSvg = (x: number, y: number) => ({ x: x * s + offsetX, y: y * s + offsetY });

  const numberInput = (value: number, onChange: (v: number) => void, label: string) => (
    <label className="flex items-center gap-1 text-[11px] text-gray-500">
      {label}
      <input
        type="number"
        min={MIN_BOX}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
        className="w-16 px-1.5 py-1 rounded border border-gray-200 bg-gray-50 text-xs text-gray-800 font-mono focus:outline-none focus:ring-2 focus:ring-orange-200"
      />
    </label>
  );

  return (
    <div className="h-full flex flex-col overflow-hidden">
      {/* Toolbar */}
      <div className="flex flex-wrap items-center justify-between gap-2 px-4 py-2 bg-white border-b border-gray-200">
        <div className="flex items-center gap-3">
          <h2 className="text-sm font-bold text-gray-800">Car Size</h2>
          <span className="text-xs text-gray-400">
            Fit the green box around your smallest car and the red box around your largest. Drag to move, corner to resize.
          </span>
          {dirty && (
            <span className="px-2 py-0.5 bg-amber-50 text-amber-700 text-[10px] font-semibold rounded-full ring-1 ring-amber-200">
              Unsaved changes
            </span>
          )}
        </div>
        <div className="flex items-center gap-2">
          <button
            onClick={() => applyConfig(saved)}
            className="flex items-center gap-1 px-2.5 py-1.5 rounded-lg text-xs font-medium bg-gray-100 hover:bg-gray-200 text-gray-600 transition-all"
            title="Discard changes and go back to the saved size"
          >
            <RotateCcw className="w-3.5 h-3.5" />
            Revert
          </button>
          <button
            onClick={handleSave}
            disabled={!dirty || saving || invalid !== null}
            className={`flex items-center gap-1 px-3 py-1.5 rounded-lg text-xs font-semibold transition-all ${
              dirty && !invalid
                ? "bg-orange-500 text-white hover:bg-orange-600 shadow-sm"
                : "bg-gray-100 text-gray-400 cursor-not-allowed"
            }`}
          >
            <Save className="w-3.5 h-3.5" />
            {saving ? "Saving..." : "Save"}
          </button>
        </div>
      </div>

      {/* Exact values */}
      <div className="flex flex-wrap items-center gap-x-6 gap-y-2 px-4 py-2 bg-white border-b border-gray-200">
        {(["small", "large"] as Which[]).map((which) => {
          const d = dims(boxes[which]);
          return (
            <div key={which} className="flex items-center gap-2">
              <span className="text-[11px] font-bold" style={{ color: BOX_STYLE[which].color }}>
                {BOX_STYLE[which].label}
              </span>
              {numberInput(d.length, (v) => setSide(which, "length", v), "Length")}
              {numberInput(d.width, (v) => setSide(which, "width", v), "Width")}
            </div>
          );
        })}
        <label className="flex items-center gap-1 text-[11px] text-gray-500">
          Tolerance
          <input
            type="number"
            min={0}
            max={90}
            step={5}
            value={Math.round(tolerance * 100)}
            onChange={(e) => {
              const v = Number(e.target.value);
              if (Number.isFinite(v)) {
                setTolerance(Math.max(0, Math.min(90, v)) / 100);
                setDirty(true);
              }
            }}
            className="w-14 px-1.5 py-1 rounded border border-gray-200 bg-gray-50 text-xs text-gray-800 font-mono focus:outline-none focus:ring-2 focus:ring-orange-200"
          />
          %
        </label>
        <span className="text-[11px] text-gray-500">
          Detected as a car: length <b className="font-mono">{accept.lengthLo}–{accept.lengthHi}px</b>, width{" "}
          <b className="font-mono">{accept.widthLo}–{accept.widthHi}px</b>
        </span>
        {(invalid || error) && <span className="text-[11px] font-semibold text-red-600">{invalid ?? error}</span>}
      </div>

      {/* Video + boxes */}
      <div ref={containerRef} className="flex-1 relative bg-black overflow-hidden">
        <img src={getVideoFeedUrl()} alt="Camera Feed" className="w-full h-full object-contain" draggable={false} />
        <svg ref={svgRef} className="absolute inset-0 w-full h-full" style={{ cursor: dragging ? "grabbing" : "default" }}>
          {(["small", "large"] as Which[]).map((which) => {
            const [x, y, w, h] = boxes[which];
            const { color, label } = BOX_STYLE[which];
            const tl = toSvg(x, y);
            const br = toSvg(x + w, y + h);
            // Dashed outline: the size actually accepted once tolerance is applied.
            const f = which === "small" ? 1 - tolerance : 1 + tolerance;
            const cx = x + w / 2;
            const cy = y + h / 2;
            const ttl = toSvg(cx - (w * f) / 2, cy - (h * f) / 2);
            const tbr = toSvg(cx + (w * f) / 2, cy + (h * f) / 2);
            const d = dims(boxes[which]);
            return (
              <g key={which}>
                <rect
                  x={ttl.x}
                  y={ttl.y}
                  width={tbr.x - ttl.x}
                  height={tbr.y - ttl.y}
                  fill="none"
                  stroke={color}
                  strokeWidth={1}
                  strokeDasharray="4 3"
                  pointerEvents="none"
                />
                <rect
                  x={tl.x}
                  y={tl.y}
                  width={br.x - tl.x}
                  height={br.y - tl.y}
                  fill={color}
                  fillOpacity={0.15}
                  stroke={color}
                  strokeWidth={2}
                  cursor="move"
                  onMouseDown={(e) => handleMouseDown(e, which, "move")}
                />
                <text x={tl.x} y={tl.y - 6} fill={color} fontSize={12} fontWeight="bold" pointerEvents="none">
                  {label}: {d.length}×{d.width}px
                </text>
                <rect
                  x={br.x - 5}
                  y={br.y - 5}
                  width={10}
                  height={10}
                  fill="white"
                  stroke={color}
                  strokeWidth={2}
                  cursor="nwse-resize"
                  onMouseDown={(e) => handleMouseDown(e, which, "resize")}
                />
              </g>
            );
          })}
        </svg>
      </div>
    </div>
  );
}
