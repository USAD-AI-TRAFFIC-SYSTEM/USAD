import { useEffect, useRef, useState, useCallback } from "react";
import { Save, RotateCcw, Eye, EyeOff } from "lucide-react";
import { getVideoFeedUrl } from "../../utils/api";

interface Point {
  x: number;
  y: number;
}

interface LaneData {
  name: string;
  region: [number, number][];
  stop_line: [number, number][];
  direction: string;
  arduino_cmd: string;
}

interface LaneConfig {
  lanes: Record<string, LaneData>;
  intersection_center: [number, number][];
  camera_width: number;
  camera_height: number;
}

const LANE_COLORS: Record<string, string> = {
  LANE1: "#22c55e", // Green - North
  LANE2: "#ef4444", // Red - South
  LANE3: "#3b82f6", // Blue - East
  LANE4: "#06b6d4", // Cyan - West
};

const INTERSECTION_COLOR = "#a855f7"; // Purple
const STOP_LINE_COLOR = "#eab308"; // Yellow
const HANDLE_RADIUS = 8;

/**
 * Identifies which lane region points correspond to intersection corners.
 * The intersection has 4 corners. Each corner is shared by 2 adjacent lanes.
 * Convention (based on default config):
 *   intersection[0] = top-left  -> LANE1 region[3], LANE4 region[3]
 *   intersection[1] = top-right -> LANE1 region[2], LANE3 region[3]
 *   intersection[2] = bot-right -> LANE2 region[2], LANE3 region[2]
 *   intersection[3] = bot-left  -> LANE2 region[3], LANE4 region[2]
 */
const CORNER_LINKS: { lane: string; regionIdx: number }[][] = [
  // intersection corner 0 (top-left)
  [{ lane: "LANE1", regionIdx: 3 }, { lane: "LANE4", regionIdx: 3 }],
  // intersection corner 1 (top-right)
  [{ lane: "LANE1", regionIdx: 2 }, { lane: "LANE3", regionIdx: 3 }],
  // intersection corner 2 (bot-right)
  [{ lane: "LANE2", regionIdx: 2 }, { lane: "LANE3", regionIdx: 2 }],
  // intersection corner 3 (bot-left)
  [{ lane: "LANE2", regionIdx: 3 }, { lane: "LANE4", regionIdx: 2 }],
];

/**
 * Stop lines are derived from intersection corners.
 * Each lane's stop line is an edge of the intersection square.
 */
const STOP_LINE_FROM_INTERSECTION: Record<string, [number, number]> = {
  LANE1: [0, 1], // North: top-left to top-right
  LANE2: [3, 2], // South: bot-left to bot-right
  LANE3: [1, 2], // East: top-right to bot-right
  LANE4: [0, 3], // West: top-left to bot-left
};

/** Region point indices that are "outer" (not linked to intersection) per lane */
const OUTER_INDICES: Record<string, number[]> = {
  LANE1: [0, 1], // top two points (far from intersection)
  LANE2: [0, 1], // bottom two points (far from intersection)
  LANE3: [0, 1], // right two points (far from intersection)
  LANE4: [0, 1], // left two points (far from intersection)
};

export default function LaneCalibrator() {
  const containerRef = useRef<HTMLDivElement>(null);
  const svgRef = useRef<SVGSVGElement>(null);
  const [config, setConfig] = useState<LaneConfig | null>(null);
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [showOverlay, setShowOverlay] = useState(true);
  const [selectedLane, setSelectedLane] = useState<string | null>(null);
  const [dragging, setDragging] = useState<{
    type: "intersection" | "outer" | "stop_line";
    lane?: string;
    index: number;
  } | null>(null);
  const [dirty, setDirty] = useState(false);
  const [imgSize, setImgSize] = useState({ width: 1280, height: 720 });

  // Load current config
  useEffect(() => {
    fetch("/api/config/lanes")
      .then((r) => r.json())
      .then((data: LaneConfig) => {
        setConfig(data);
        setImgSize({ width: data.camera_width, height: data.camera_height });
        setLoading(false);
      })
      .catch(() => setLoading(false));
  }, []);

  // Compute scale from video native resolution to displayed size
  const getScale = useCallback(() => {
    if (!containerRef.current) return { sx: 1, sy: 1, offsetX: 0, offsetY: 0 };
    const rect = containerRef.current.getBoundingClientRect();
    const containerAspect = rect.width / rect.height;
    const videoAspect = imgSize.width / imgSize.height;

    let displayW: number, displayH: number, offsetX: number, offsetY: number;
    if (containerAspect > videoAspect) {
      displayH = rect.height;
      displayW = rect.height * videoAspect;
      offsetX = (rect.width - displayW) / 2;
      offsetY = 0;
    } else {
      displayW = rect.width;
      displayH = rect.width / videoAspect;
      offsetX = 0;
      offsetY = (rect.height - displayH) / 2;
    }

    return { sx: displayW / imgSize.width, sy: displayH / imgSize.height, offsetX, offsetY };
  }, [imgSize]);

  const toSvg = useCallback(
    (pt: [number, number]): Point => {
      const { sx, sy, offsetX, offsetY } = getScale();
      return { x: pt[0] * sx + offsetX, y: pt[1] * sy + offsetY };
    },
    [getScale]
  );

  const toNative = useCallback(
    (pt: Point): [number, number] => {
      const { sx, sy, offsetX, offsetY } = getScale();
      return [
        Math.max(0, Math.min(imgSize.width, Math.round((pt.x - offsetX) / sx))),
        Math.max(0, Math.min(imgSize.height, Math.round((pt.y - offsetY) / sy))),
      ];
    },
    [getScale, imgSize]
  );

  const handleMouseDown = (
    e: React.MouseEvent,
    type: "intersection" | "outer" | "stop_line",
    index: number,
    lane?: string
  ) => {
    e.preventDefault();
    e.stopPropagation();
    setDragging({ type, lane, index });
  };

  const handleMouseMove = useCallback(
    (e: MouseEvent) => {
      if (!dragging || !config || !svgRef.current) return;

      const rect = svgRef.current.getBoundingClientRect();
      const svgPt: Point = { x: e.clientX - rect.left, y: e.clientY - rect.top };
      const native = toNative(svgPt);

      const newConfig = JSON.parse(JSON.stringify(config)) as LaneConfig;

      if (dragging.type === "intersection") {
        // Move intersection corner + all linked lane region points + stop lines
        newConfig.intersection_center[dragging.index] = native;
        for (const link of CORNER_LINKS[dragging.index]) {
          if (newConfig.lanes[link.lane]) {
            newConfig.lanes[link.lane].region[link.regionIdx] = native;
          }
        }
        // Update stop lines derived from intersection corners
        for (const [laneKey, [idx0, idx1]] of Object.entries(STOP_LINE_FROM_INTERSECTION)) {
          if (newConfig.lanes[laneKey]) {
            newConfig.lanes[laneKey].stop_line = [
              newConfig.intersection_center[idx0],
              newConfig.intersection_center[idx1],
            ];
          }
        }
      } else if (dragging.type === "outer" && dragging.lane) {
        // Move an outer lane point (not linked to intersection)
        const outerIdx = OUTER_INDICES[dragging.lane][dragging.index];
        newConfig.lanes[dragging.lane].region[outerIdx] = native;
      } else if (dragging.type === "stop_line" && dragging.lane) {
        newConfig.lanes[dragging.lane].stop_line[dragging.index] = native;
      }

      setConfig(newConfig);
      setDirty(true);
    },
    [dragging, config, toNative]
  );

  const handleMouseUp = useCallback(() => {
    setDragging(null);
  }, []);

  useEffect(() => {
    if (dragging) {
      window.addEventListener("mousemove", handleMouseMove);
      window.addEventListener("mouseup", handleMouseUp);
      return () => {
        window.removeEventListener("mousemove", handleMouseMove);
        window.removeEventListener("mouseup", handleMouseUp);
      };
    }
  }, [dragging, handleMouseMove, handleMouseUp]);

  // Save
  const handleSave = async () => {
    if (!config) return;
    setSaving(true);
    try {
      const res = await fetch("/api/config/lanes", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          lanes: config.lanes,
          intersection_center: config.intersection_center,
        }),
      });
      const data = await res.json();
      if (data.ok) {
        setDirty(false);
        alert("Lane configuration saved successfully!");
      } else {
        alert("Failed to save: " + (data.error || "Unknown error"));
      }
    } catch {
      alert("Network error saving config");
    }
    setSaving(false);
  };

  // Reset to original default coordinates
  const handleReset = () => {
    if (!confirm("Reset to original default coordinates? Your current adjustments will be lost on screen (but not saved until you click Save).")) return;
    setLoading(true);
    fetch("/api/config/lanes/defaults")
      .then((r) => r.json())
      .then((data: LaneConfig) => {
        setConfig(data);
        setDirty(true);
        setLoading(false);
      })
      .catch(() => setLoading(false));
  };

  if (loading || !config) {
    return (
      <div className="flex items-center justify-center h-full text-gray-500">
        Loading calibration data...
      </div>
    );
  }

  return (
    <div className="h-full flex flex-col overflow-hidden">
      {/* Toolbar */}
      <div className="flex items-center justify-between px-4 py-2 bg-white border-b border-gray-200">
        <div className="flex items-center gap-3">
          <h2 className="text-sm font-bold text-gray-800">Lane Calibration</h2>
          <span className="text-xs text-gray-400">
            Drag handles to adjust. Center corners move linked lanes.
          </span>
          {dirty && (
            <span className="px-2 py-0.5 bg-amber-50 text-amber-700 text-[10px] font-semibold rounded-full ring-1 ring-amber-200">
              Unsaved changes
            </span>
          )}
        </div>
        <div className="flex items-center gap-2">
          {/* Lane selector pills */}
          <div className="flex gap-1 mr-3">
            {Object.entries(config.lanes).map(([key, lane]) => (
              <button
                key={key}
                onClick={() => setSelectedLane(selectedLane === key ? null : key)}
                className={`px-2 py-1 rounded text-[10px] font-bold transition-all ${
                  selectedLane === key
                    ? "ring-2 ring-offset-1 text-white"
                    : "text-gray-600 bg-gray-100 hover:bg-gray-200"
                }`}
                style={selectedLane === key ? { backgroundColor: LANE_COLORS[key] } : {}}
              >
                {lane.name}
              </button>
            ))}
          </div>

          <button
            onClick={() => setShowOverlay(!showOverlay)}
            className="flex items-center gap-1 px-2.5 py-1.5 rounded-lg text-xs font-medium bg-gray-100 hover:bg-gray-200 text-gray-600 transition-all"
          >
            {showOverlay ? <Eye className="w-3.5 h-3.5" /> : <EyeOff className="w-3.5 h-3.5" />}
          </button>

          <button
            onClick={handleReset}
            className="flex items-center gap-1 px-2.5 py-1.5 rounded-lg text-xs font-medium bg-gray-100 hover:bg-gray-200 text-gray-600 transition-all"
            title="Reset to defaults"
          >
            <RotateCcw className="w-3.5 h-3.5" />
            Reset
          </button>

          <button
            onClick={handleSave}
            disabled={!dirty || saving}
            className={`flex items-center gap-1 px-3 py-1.5 rounded-lg text-xs font-semibold transition-all ${
              dirty
                ? "bg-orange-500 text-white hover:bg-orange-600 shadow-sm"
                : "bg-gray-100 text-gray-400 cursor-not-allowed"
            }`}
          >
            <Save className="w-3.5 h-3.5" />
            {saving ? "Saving..." : "Save"}
          </button>
        </div>
      </div>

      {/* Video + Overlay */}
      <div ref={containerRef} className="flex-1 relative bg-black overflow-hidden">
        <img
          src={getVideoFeedUrl()}
          alt="Camera Feed"
          className="w-full h-full object-contain"
          draggable={false}
        />

        {showOverlay && (
          <svg
            ref={svgRef}
            className="absolute inset-0 w-full h-full"
            style={{ cursor: dragging ? "grabbing" : "default" }}
          >
            {/* Lane region polygons */}
            {Object.entries(config.lanes).map(([key, lane]) => {
              const pts = lane.region.map(toSvg);
              const isSelected = selectedLane === key || selectedLane === null;
              const opacity = isSelected ? 1 : 0.3;

              return (
                <g key={key} opacity={opacity}>
                  <polygon
                    points={pts.map((p) => `${p.x},${p.y}`).join(" ")}
                    fill={LANE_COLORS[key]}
                    fillOpacity={0.1}
                    stroke={LANE_COLORS[key]}
                    strokeWidth={2}
                  />
                  {/* Lane label */}
                  <text
                    x={pts.reduce((s, p) => s + p.x, 0) / pts.length}
                    y={pts.reduce((s, p) => s + p.y, 0) / pts.length}
                    textAnchor="middle"
                    fill={LANE_COLORS[key]}
                    fontSize={14}
                    fontWeight="bold"
                  >
                    {lane.name}
                  </text>
                  {/* Outer handles (the 2 far-edge points per lane) */}
                  {OUTER_INDICES[key]?.map((regionIdx, outerI) => {
                    const p = toSvg(lane.region[regionIdx]);
                    return (
                      <circle
                        key={`${key}-outer-${outerI}`}
                        cx={p.x}
                        cy={p.y}
                        r={HANDLE_RADIUS}
                        fill="white"
                        stroke={LANE_COLORS[key]}
                        strokeWidth={2.5}
                        cursor="grab"
                        onMouseDown={(e) => handleMouseDown(e, "outer", outerI, key)}
                      />
                    );
                  })}
                  {/* Stop line (auto-follows intersection, no handles) */}
                  {lane.stop_line.length === 2 && (() => {
                    const sl = lane.stop_line.map(toSvg);
                    return (
                      <line
                        x1={sl[0].x}
                        y1={sl[0].y}
                        x2={sl[1].x}
                        y2={sl[1].y}
                        stroke={STOP_LINE_COLOR}
                        strokeWidth={3}
                        strokeDasharray="6 3"
                      />
                    );
                  })()}
                </g>
              );
            })}

            {/* Intersection center polygon + shared corner handles */}
            {(() => {
              const pts = config.intersection_center.map(toSvg);
              return (
                <g>
                  <polygon
                    points={pts.map((p) => `${p.x},${p.y}`).join(" ")}
                    fill={INTERSECTION_COLOR}
                    fillOpacity={0.06}
                    stroke={INTERSECTION_COLOR}
                    strokeWidth={2}
                    strokeDasharray="4 4"
                  />
                  {/* 4 shared corner handles */}
                  {pts.map((p, i) => (
                    <circle
                      key={`inter-${i}`}
                      cx={p.x}
                      cy={p.y}
                      r={HANDLE_RADIUS + 1}
                      fill={INTERSECTION_COLOR}
                      stroke="white"
                      strokeWidth={2.5}
                      cursor="grab"
                      onMouseDown={(e) => handleMouseDown(e, "intersection", i)}
                    />
                  ))}
                </g>
              );
            })()}
          </svg>
        )}
      </div>

      {/* Coordinate readout */}
      <div className="px-4 py-2 bg-white border-t border-gray-200 text-[10px] text-gray-400 font-mono flex gap-6">
        {selectedLane && config.lanes[selectedLane] && (
          <>
            <span>
              Region: {config.lanes[selectedLane].region.map((p) => `(${p[0]},${p[1]})`).join(" ")}
            </span>
            <span>
              Stop: {config.lanes[selectedLane].stop_line.map((p) => `(${p[0]},${p[1]})`).join(" ")}
            </span>
          </>
        )}
        {!selectedLane && (
          <span>
            Intersection: {config.intersection_center.map((p) => `(${p[0]},${p[1]})`).join(" ")} | Drag purple corners to reshape the cross.
          </span>
        )}
      </div>
    </div>
  );
}
