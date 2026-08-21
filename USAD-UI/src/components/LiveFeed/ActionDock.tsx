import { Zap, RotateCcw, Eraser, Camera, Maximize } from "lucide-react";
import { controlAction } from "../../utils/api";

const ACTIONS = [
  { key: "A", label: "Auto",       icon: <Zap className="w-3.5 h-3.5" />,                       action: () => controlAction("auto"),           style: "bg-orange-100 text-orange-700 hover:bg-orange-200 ring-1 ring-orange-300" },
  { key: "1", label: "North",      icon: <span className="text-[11px] font-black">N</span>,      action: () => controlAction("lane/LANE1"),      style: "bg-teal-50 text-teal-700 hover:bg-teal-100 ring-1 ring-teal-200" },
  { key: "2", label: "South",      icon: <span className="text-[11px] font-black">S</span>,      action: () => controlAction("lane/LANE2"),      style: "bg-teal-50 text-teal-700 hover:bg-teal-100 ring-1 ring-teal-200" },
  { key: "3", label: "East",       icon: <span className="text-[11px] font-black">E</span>,      action: () => controlAction("lane/LANE3"),      style: "bg-teal-50 text-teal-700 hover:bg-teal-100 ring-1 ring-teal-200" },
  { key: "4", label: "West",       icon: <span className="text-[11px] font-black">W</span>,      action: () => controlAction("lane/LANE4"),      style: "bg-teal-50 text-teal-700 hover:bg-teal-100 ring-1 ring-teal-200" },
  { key: "R", label: "Reset",      icon: <RotateCcw className="w-3.5 h-3.5" />,                  action: () => controlAction("reset"),          style: "bg-amber-50 text-amber-700 hover:bg-amber-100 ring-1 ring-amber-200" },
  { key: "B", label: "BG Reset",   icon: <Eraser className="w-3.5 h-3.5" />,                     action: () => controlAction("reset-bg"),       style: "bg-amber-50 text-amber-700 hover:bg-amber-100 ring-1 ring-amber-200" },
  { key: "C", label: "Camera",     icon: <Camera className="w-3.5 h-3.5" />,                     action: () => controlAction("cycle-camera"),   style: "bg-teal-50 text-teal-700 hover:bg-teal-100 ring-1 ring-teal-200" },
  { key: "F", label: "Fullscreen", icon: <Maximize className="w-3.5 h-3.5" />,                   action: () => {
    if (!document.fullscreenElement) document.documentElement.requestFullscreen();
    else document.exitFullscreen();
  }, style: "bg-gray-100 text-gray-600 hover:bg-gray-200 ring-1 ring-gray-300" },
];

export default function ActionDock() {
  return (
    <div className="absolute bottom-5 left-1/2 -translate-x-1/2 z-10">
      <p className="text-center text-[9px] font-bold text-white/60 uppercase tracking-widest mb-1.5 drop-shadow">
        Control Keys
      </p>
      <div className="glass-card flex items-center gap-1.5 px-3 py-2">
        {ACTIONS.map((a) => (
          <button
            key={a.key}
            onClick={a.action}
            title={`${a.label} [${a.key}]`}
            className={`flex items-center gap-1.5 px-2.5 py-1.5 rounded-lg text-xs font-semibold transition-all duration-200 active:scale-95 ${a.style}`}
          >
            {a.icon}
            <span className="hidden sm:inline">{a.label}</span>
            <kbd className="text-[9px] opacity-40 font-mono">[{a.key}]</kbd>
          </button>
        ))}
      </div>
    </div>
  );
}
