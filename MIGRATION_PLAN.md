# USAD System UI Modernization & Migration Plan (Tkinter → React Desktop)

## 1. Overview & Objectives

This document details the complete, step-by-step implementation plan for modernizing the **USAD (Urban Smart Adaptive Dispatcher)** frontend.

### Core Objectives:
1. **100% Core Logic Preservation**: Absolutely zero changes to AI detection math, YOLO tracking, background subtraction, accident detection algorithms, red-light violation checks, EasyOCR license plate reader, Arduino PySerial communication, or CSV logging.
2. **Eliminate Tkinter Bottlenecks**: Fix UI thread freezing during OCR/model inference, eradicate PIL image-conversion overhead, and eliminate window-placement glitches.
3. **Unified Single-Window Interface**: Merge the live camera HUD (`app.py`) and the analytics dashboard (`dashboard.py`) into a sleek, responsive React desktop application with instant tab switching.
4. **100% Local & Offline Execution**: Zero external cloud dependencies. Operates entirely on `localhost`.
5. **Seamless Standalone Distribution**: Can be packaged into a single standalone `.exe` using PyInstaller and `pywebview` / Tauri.

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      UNTOUCHED BACKEND ENGINE                           │
│  main.py (USAD Engine)                                                  │
│  ├── vehicle_detector.py        ├── license_plate_detector.py           │
│  ├── accident_detector.py       ├── traffic_controller.py (Arduino)     │
│  ├── violation_detector.py      ├── event_logger.py                     │
│  └── emergency_notifier.py      └── config.py                           │
└────────────────────────────────────▲────────────────────────────────────┘
                                     │ Direct Python Method Calls
┌────────────────────────────────────▼────────────────────────────────────┐
│                    LOCAL BRIDGE (FastAPI + AsyncIO)                     │
│  server.py                                                              │
│  ├── GET /api/video/feed         (MJPEG / WebSocket Frame Stream)       │
│  ├── WS  /ws/telemetry           (30Hz Real-Time JSON Telemetry)        │
│  ├── POST /api/control/{action}  (Auto, Reset, BG Reset, Lane Override) │
│  ├── POST /api/camera/cycle      (Switch Cam 1 <-> Cam 2)               │
│  └── GET  /api/logs/{type}       (Violations, Accidents, Traffic Events)│
└────────────────────────────────────▲────────────────────────────────────┘
                                     │ Localhost HTTP / WS
┌────────────────────────────────────▼────────────────────────────────────┐
│                   MODERN REACT DESKTOP FRONTEND                         │
│  USAD-UI (React + Vite + TailwindCSS + Lucide + Recharts)               │
│  ├── Live Traffic HUD View       ├── Controls & Action Dock             │
│  │   ├── Video Canvas Stream     │   ├── [Q]uit, [R]eset, [B]g Reset    │
│  │   ├── Active Camera Badge     │   ├── [A]uto Mode, [1-4] Lane Force  │
│  │   ├── Arduino Status Badge    │   └── [C]ycle Camera, [F]ullscreen   │
│  │   └── Glowing Stoplight HUD   │                                      │
│  │       (North, South, East,    ├── Analytics & Reports Dashboard      │
│  │        West Timers & LEDs)    │   ├── Hourly Incident Bar Charts     │
│  │                               │   ├── Per-Lane Vehicle Distribution  │
│  └── Toast & Audio Alert System  │   └── Filterable & Searchable Tables │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Communication Layer & API Specification

The local FastAPI server will run on `http://127.0.0.1:8000` (or dynamically selected port) and expose the following endpoints:

### 3.1 Video Streaming
- **`GET /api/video/feed`**: Returns a `multipart/x-mixed-replace; boundary=frame` MJPEG stream. The browser decodes this natively via hardware acceleration `<img src="/api/video/feed" />` or a `<canvas>` element with zero PIL conversion overhead.

### 3.2 Real-Time Telemetry WebSocket
- **`WS /ws/telemetry`**: Pushes JSON payloads at 30Hz:
```json
{
  "fps": 29.8,
  "camera_source": 1,
  "arduino_connected": true,
  "software_auto_mode": true,
  "current_active_lane": "LANE1",
  "current_phase": "GREEN",
  "phase_time_remaining": 14.5,
  "total_vehicles": 8,
  "lane_counts": {
    "LANE1": 4,
    "LANE2": 2,
    "LANE3": 1,
    "LANE4": 1,
    "INTERSECTION": 0
  },
  "lane_states": {
    "LANE1": "CONGESTED",
    "LANE2": "CONGESTED",
    "LANE3": "NON-CONGESTED",
    "LANE4": "NON-CONGESTED",
    "INTERSECTION": "EMPTY"
  },
  "lane_signals": {
    "LANE1": "GREEN",
    "LANE2": "RED",
    "LANE3": "RED",
    "LANE4": "RED"
  },
  "adaptive_green_durations": {
    "LANE1": 8,
    "LANE2": 8,
    "LANE3": 3,
    "LANE4": 3
  },
  "active_accidents_count": 0,
  "active_stopped_cars_count": 0,
  "total_violations_session": 2,
  "emergency_notifications_count": 0,
  "detected_license_plates": [
    { "text": "ABC1234", "confidence": 0.94 }
  ]
}
```

### 3.3 Control API
- **`POST /api/control/auto`**: Enables automatic signal timing (`usad.traffic_controller.set_auto_mode()` / `software_auto_mode = True`).
- **`POST /api/control/lane/{lane_key}`**: Activates manual override for `LANE1`, `LANE2`, `LANE3`, or `LANE4`.
- **`POST /api/control/reset`**: Resets vehicle, accident, violation, and emergency detector states.
- **`POST /api/control/reset-bg`**: Calls `usad.vehicle_detector.reset(reset_background=True)`.
- **`POST /api/control/cycle-camera`**: Triggers `usad.cycle_camera()`.
- **`POST /api/control/shutdown`**: Gracefully prints session statistics, releases hardware, and terminates the application.

### 3.4 Data & Analytics API
- **`GET /api/logs/summary`**: Returns total counts of violations, accidents, and dispatches.
- **`GET /api/logs/violations`**: Returns parsed records from `violations.csv`.
- **`GET /api/logs/accidents`**: Returns parsed records from `accidents.csv`.
- **`GET /api/logs/traffic`**: Returns parsed records from `traffic_events.csv`.
- **`GET /api/logs/export/{type}`**: Generates an exportable CSV or text analytics report.

---

## 4. Frontend Component Hierarchy & UI Design

### 4.1 Visual Design Tokens
- **Theme**: Premium High-Tech Dark Mode (`#0a0d14` background, `#121824` cards, `#1e293b` borders).
- **Typography**: `Inter` / `Outfit` font family.
- **Stoplight Colors**:
  - Red: `#ef4444` (with `rgba(239, 68, 68, 0.4)` glowing drop-shadow)
  - Yellow: `#f59e0b` (with `rgba(245, 158, 11, 0.4)` glowing drop-shadow)
  - Green: `#10b981` (with `rgba(16, 185, 129, 0.4)` glowing drop-shadow)
  - Inactive LED: `#262e3d`
- **Glassmorphism**: `backdrop-blur-md bg-slate-900/80 border border-slate-800/80`.

### 4.2 Component Tree
```
App
├── NavigationHeader
│   ├── Logo & Brand ("USAD · Urban Smart Adaptive Dispatcher")
│   ├── Mode Tabs ("Live Camera Feed" | "Analytics & Logs" | "System Config")
│   ├── Connection Badges (Camera Source 1/2, Arduino Connected/Simulation, FPS)
│   └── System Clock & Shutdown Button
│
├── LiveFeedTab (Default View)
│   ├── VideoPlayerCanvas (Live Stream Feed with Overlay Support)
│   ├── FloatingHUDPanel (Right Side Glassmorphic Card)
│   │   ├── ActiveSignalCard (Active Lane Name + Animated Radial Timer Countdown)
│   │   ├── LaneStoplightsList (North, South, East, West with live R/Y/G LEDs)
│   │   ├── CongestionMatrix (Vehicle Counts & Congestion Badges per Lane)
│   │   └── QuickStatsGrid (Total Vehicles, Accidents, Violations, Emergency Calls)
│   ├── FloatingActionDock (Bottom Bar)
│   │   ├── [A] Auto Mode Switch
│   │   ├── [1-4] Manual Lane Buttons
│   │   ├── [R] Reset Detectors
│   │   ├── [B] Reset Background
│   │   ├── [C] Cycle Camera
│   │   └── [F] Toggle Fullscreen
│   └── AlertBanner & ToastManager (Accident Detected / Violation Alerts)
│
├── AnalyticsDashboardTab
│   ├── KPIOverviewCards (Total Violations, Total Accidents, Emergency Dispatches)
│   ├── HourlyIncidentsChart (Recharts Bar & Line Chart of hourly incidents)
│   ├── LaneCongestionDistribution (Donut / Radial Chart of Traffic Flow)
│   └── LogRecordsViewer
│       ├── Sub-tabs (Violations Log | Accidents Log | Traffic Events)
│       ├── Search & Filter Toolbar (Date range, Lane filter, Violation type)
│       ├── Paginated / Virtualized Data Table
│       └── CSV Export Button
│
└── SystemConfigTab (Visual Lane Calibrator)
    ├── Interactive Canvas with Draggable Polygon Handles
    ├── Lane Boundary Coordinate Inspector
    └── Timing Configuration Sliders
```

---

## 5. File Structure Blueprint

```
USAD_latest/
├── USAD-Model/
│   ├── main.py                     # [UNCHANGED] Core AI & Control Engine
│   ├── vehicle_detector.py         # [UNCHANGED] Vehicle detection & tracking
│   ├── accident_detector.py        # [UNCHANGED] Accident detection
│   ├── violation_detector.py       # [UNCHANGED] Violation detection
│   ├── license_plate_detector.py   # [UNCHANGED] OCR plate detector
│   ├── traffic_controller.py       # [UNCHANGED] Arduino serial comms
│   ├── event_logger.py             # [UNCHANGED] CSV logger & report generator
│   ├── emergency_notifier.py       # [UNCHANGED] SMS/Email notifier
│   ├── config.py                   # [UNCHANGED] Configuration settings
│   │
│   ├── server.py                   # [NEW] FastAPI Bridge & WebSocket Server
│   ├── desktop_app.py              # [NEW] Desktop Launcher (pywebview / web launcher)
│   │
│   ├── app.py                      # [REPLACED/DEPRECATED] Points to desktop_app.py
│   └── dashboard.py                # [REPLACED/DEPRECATED] Integrated into React UI
│
├── USAD-UI/                        # [NEW] Modern React Frontend Project
│   ├── package.json
│   ├── vite.config.ts
│   ├── tailwind.config.js
│   ├── tsconfig.json
│   ├── index.html
│   └── src/
│       ├── main.tsx
│       ├── App.tsx
│       ├── types/telemetry.ts
│       ├── hooks/useTelemetry.ts
│       ├── hooks/useKeyboardShortcuts.ts
│       ├── components/
│       │   ├── Header/Navbar.tsx
│       │   ├── LiveFeed/
│       │   │   ├── VideoCanvas.tsx
│       │   │   ├── FloatingHUD.tsx
│       │   │   ├── StoplightItem.tsx
│       │   │   ├── ActionDock.tsx
│       │   │   └── AlertToast.tsx
│       │   ├── Dashboard/
│       │   │   ├── StatCard.tsx
│       │   │   ├── IncidentChart.tsx
│       │   │   ├── TrafficDistribution.tsx
│       │   │   └── LogTable.tsx
│       │   └── Calibration/
│       │       └── LaneEditor.tsx
│       └── utils/api.ts
│
├── build_app.py                    # [UPDATED] PyInstaller build config
└── SETUP.bat                       # [UPDATED] Build script
```

---

## 6. Detailed Implementation Steps

### Phase 1: Local FastAPI Server Bridge (`server.py`)
1. Create `server.py` in `USAD-Model/`.
2. Import `from main import USAD`.
3. Initialize the `USAD` instance as a singleton (`usad = USAD()`, `usad.initialize_camera()`, `usad.initialize_arduino()`).
4. Set `usad.show_cv_panel = False` so OpenCV does not burn extra CPU drawing redundant HUD elements onto the raw feed.
5. Create a dedicated frame grabber background worker:
   - Continuously grabs frames using `usad.cap.read()`.
   - Runs `processed = usad.process_frame(frame)`.
   - Encodes frame as JPEG (`cv2.imencode('.jpg', processed, [cv2.IMWRITE_JPEG_QUALITY, 85])`).
   - Yields multipart MJPEG bytes for `/api/video/feed`.
6. Implement the 30Hz WebSocket loop (`/ws/telemetry`) broadcasting engine state.
7. Implement control REST endpoints mapping directly to engine methods.

### Phase 2: React Frontend Scaffolding (`USAD-UI/`)
1. Initialize Vite React TypeScript project: `npm create vite@latest USAD-UI -- --template react-ts`.
2. Install dependencies:
   - `lucide-react` (icons)
   - `recharts` (charts & graphs)
   - `clsx`, `tailwind-merge` (styling utilities)
   - `tailwindcss`, `postcss`, `autoprefixer`
3. Configure TailwindCSS with custom glowing colors and dark glassmorphic tokens.
4. Build the custom telemetry hook `useTelemetry()` with automatic WebSocket reconnection and state buffering.
5. Implement global keyboard listener hook `useKeyboardShortcuts()` handling `[Q]`, `[R]`, `[B]`, `[A]`, `[1]`, `[2]`, `[3]`, `[4]`, `[C]`, `[F]`.

### Phase 3: Building the Live HUD & Stoplight Components
1. Build `VideoCanvas.tsx` with responsive aspect ratio preservation, camera error placeholder, and switch transitions.
2. Build `FloatingHUD.tsx` with:
   - Glowing per-lane stoplight LEDs (North, South, East, West).
   - Dynamic animated countdown timers for the active green/yellow phase.
   - Congestion badges (`CONGESTED`, `NON-CONGESTED`, `EMPTY`).
3. Build `ActionDock.tsx` displaying the interactive key buttons with instant visual feedback and tooltip guides.
4. Build `AlertToast.tsx` displaying animated accident and violation warning banners.

### Phase 4: Building the Integrated Analytics Dashboard
1. Build `IncidentChart.tsx` consuming `violations.csv` and `accidents.csv` data to render hourly breakdown charts.
2. Build `TrafficDistribution.tsx` showing per-lane vehicle throughput.
3. Build `LogTable.tsx` featuring real-time search, column sorting, lane filter dropdowns, and one-click CSV export.

### Phase 5: Desktop Packaging (`desktop_app.py` & `build_app.py`)
1. Build static React production bundle: `npm run build` (outputs to `USAD-UI/dist`).
2. Configure FastAPI in `server.py` to serve static files from `USAD-UI/dist` on the root route `/`.
3. Create `desktop_app.py` using `pywebview` to open `http://127.0.0.1:8000` in a clean native desktop window (maximized, borderless options, custom icon).
4. Update `build_app.py` to bundle `USAD-UI/dist` as PyInstaller data files.
5. Test standalone execution via `USAD.exe`.

---

## 7. Verification & Testing Checklist

- [ ] **Video Stream Quality**: Sustained 30 FPS stream with no visual tearing or frame drops.
- [ ] **AI Detection Accuracy**: Vehicle boxes, tracking IDs, red-light violation lines, and accident markers render identically to the OpenCV wireframe.
- [ ] **Camera Cycling**: Pressing `[C]` or clicking the Camera switch button seamlessly switches between Camera 1 (Lanes) and Camera 2 (License Plates).
- [ ] **Arduino Hardware & Simulation Parity**:
  - Arduino connected -> LEDs and relays trigger as expected.
  - Arduino disconnected -> Automatic software simulation engages with countdown timers.
- [ ] **Telemetry Accuracy**: Stoplight timers, active lane indicators, and congestion counts update synchronously with video events.
- [ ] **Keyboard Controls**: All keys (`Q`, `R`, `B`, `A`, `1-4`, `C`, `F`, `S`) perform their designated backend actions.
- [ ] **CSV Logs & Analytics**: CSVs are recorded in `logs/` and accurately reflected in the dashboard tables and charts.
- [ ] **Standalone Build**: `USAD.exe` launches smoothly on Windows without requiring Node.js or Python installations.
