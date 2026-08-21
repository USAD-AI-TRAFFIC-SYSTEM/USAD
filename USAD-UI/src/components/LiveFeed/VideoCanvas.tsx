import { getVideoFeedUrl } from "../../utils/api";

export default function VideoCanvas() {
  return (
    <div className="relative w-full h-full bg-black flex items-center justify-center overflow-hidden">
      <img
        src={getVideoFeedUrl()}
        alt="USAD Live Feed"
        className="w-full h-full object-contain"
        draggable={false}
        onError={(e) => {
          // On error, show placeholder after a delay
          const img = e.currentTarget;
          setTimeout(() => {
            img.style.display = "none";
            const placeholder = img.nextElementSibling as HTMLElement;
            if (placeholder) placeholder.style.display = "flex";
          }, 500);
        }}
      />
      {/* Fallback placeholder (hidden by default) */}
      <div
        className="absolute inset-0 flex-col items-center justify-center gap-4 text-slate-500"
        style={{ display: "none" }}
      >
        <div className="w-16 h-16 rounded-full border-2 border-slate-700 flex items-center justify-center">
          <svg
            className="w-8 h-8 animate-pulse"
            fill="none"
            viewBox="0 0 24 24"
            stroke="currentColor"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth={1.5}
              d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z"
            />
          </svg>
        </div>
        <p className="text-sm">Connecting to camera feed…</p>
      </div>
    </div>
  );
}
