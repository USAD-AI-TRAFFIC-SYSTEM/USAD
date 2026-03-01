# USAD – Urban Smart Adaptive Detection

USAD is an AI‑assisted traffic management system that uses classical computer vision and adaptive control to improve safety and throughput at urban intersections.

It ingests a live video feed, detects and tracks vehicles, identifies dangerous situations (accidents and violations), and coordinates both simulated and physical traffic lights via Arduino.

## 🚦 Key Features

### Core Capabilities
- **Real‑time vehicle detection and tracking** – Background subtraction, color segmentation, and robust multi-frame tracking to follow vehicles through the intersection.
- **Accident detection** – Differentiates between stopped vehicles and collisions, tracks accident duration, and suppresses duplicate alerts across frames.
- **Emergency notifications** – Simulates SMS and phone-call style notifications to a configurable hotline (default `911`) for confirmed accidents, with cooldown control.
- **E.Y.E. (Eyeing Your Encounter)** – Rule‑based violation detection for red‑light violations.
- **License plate recognition (LPR)** – Uses EasyOCR in a dedicated worker thread so OCR never blocks the video loop, with template/Tesseract-based fallbacks and throttling to avoid frame drops.
- **Adaptive traffic signal control** – Adjusts green time per lane based on measured congestion while honoring configurable minimum and maximum green durations.
- **Arduino integration with graceful fallback** – Controls a physical 4‑lane traffic light controller via serial on `config.ARDUINO_PORT` (default `COM5`); automatically switches to software‑simulated signals when Arduino is not available.
- **Structured logging and analytics** – Logs all events, violations, accidents, and recognized license plates to CSV; supports summary analytics and simple text reports.

### High‑Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        USAD System                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌───────────┐    ┌─────────────┐    ┌────────────────┐     │
│  │  Camera   │───▶│   Vehicle   │───▶│   Accident     │     │
│  │  Input    │    │  Detector   │    │   Detector     │     │
│  └───────────┘    └─────────────┘    └────────────────┘     │
│                           │                    │            │
│                           ▼                    ▼            │
│                    ┌─────────────┐    ┌────────────────┐    │
│                    │  Violation  │    │   Emergency    │    │
│                    │  Detector   │    │   Notifier     │    │
│                    │   (E.Y.E.)  │    └────────────────┘    │
│                    └─────────────┘             │            │
│                           │                    │            │
│                           ▼                    ▼            │
│                    ┌─────────────────────────────────┐      │
│                    │       Event Logger              │      │
│                    │   (CSV Analytics & Reports)     │      │
│                    └─────────────────────────────────┘      │
│                                                             │
│                    ┌─────────────────────────────────┐      │
│                    │    Traffic Controller           │      │
│                    │     (Arduino Integration)       │      │
│                    └─────────────────────────────────┘      │
│                                   │                         │
└───────────────────────────────────┼──────────────────────── ┘
                                    ▼
                         ┌──────────────────┐
                         │  Arduino Traffic │
                         │  Light Hardware  │
                         └──────────────────┘
```

## 📋 Requirements

### Hardware
- **Camera** – Webcam or CCTV with a bird’s‑eye or high‑angle view of the intersection.
- **Arduino** – Board connected to `config.ARDUINO_PORT` (default `COM5`) driving a 4‑lane traffic light rig:
  - Each lane has red, yellow, and green LEDs.
  - Pin configuration is defined in `arduino/traffic_controller.ino` (see "Arduino Traffic Controller" below).
- **Host computer** – Windows PC with a USB port for the Arduino and sufficient CPU for OpenCV + EasyOCR.

### Software
- Python 3.8 or later
- Arduino IDE (for compiling and uploading the traffic‑controller sketch)
- Tesseract OCR (optional; used by some plate‑recognition paths when configured)

## 🚀 Installation

> **Note**: All commands assume your working directory is the repository root (`USAD`).

### 1. Clone the repository

```bash
git clone https://github.com/USAD-AI-TRAFFIC-SYSTEM/USAD--AI-TRAFFIC-SYSTEM.git
cd USAD
```

### 2. Create and activate a virtual environment

**Windows (PowerShell):**

```bash
python -m venv venv
./venv/Scripts/Activate.ps1
```

**Linux / macOS:**

```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Python dependencies

From the `USAD` root directory:

```bash
pip install -r requirements.txt
```

### 4. Install Tesseract OCR (optional, for LPR)

If you want license‑plate OCR:
- Download and install Tesseract from: https://github.com/tesseract-ocr/tesseract
- If Tesseract is not on your PATH, set the executable path in `USAD-Model/license_plate_detector.py`:

```python
pytesseract.pytesseract.tesseract_cmd = r"C:\\Program Files\\Tesseract-OCR\\tesseract.exe"
```

### 5. Upload the Arduino sketch

1. Open `arduino/traffic_controller.ino` in the Arduino IDE.
2. In `USAD-Model/config.py`, ensure `ARDUINO_PORT` matches the COM port used by your board (default `COM5`).
3. Select the correct board and port in the Arduino IDE.
4. Upload the sketch to the Arduino.

### 6. Configure the camera source

Edit `USAD-Model/config.py` and set:

```python
CAMERA_SOURCE = 0  # 0 for default webcam, or a video file path
```

You can also adjust `CAMERA_WIDTH`, `CAMERA_HEIGHT`, and `CAMERA_FPS` if needed.

## ⚙️ Configuration Overview

Most runtime behavior is controlled via `USAD-Model/config.py`:

- **Camera and geometry**
  - `CAMERA_SOURCE`, `CAMERA_WIDTH`, `CAMERA_HEIGHT`, `CAMERA_FPS`
  - `LANES` and `INTERSECTION_CENTER` polygons that define lane regions and the central box
- **Traffic signal timing**
  - `GREEN_TIME`, `YELLOW_TIME`, `RED_TIME`
  - `MIN_GREEN_TIME`, `MAX_GREEN_TIME`
  - `ENABLE_ADAPTIVE_TIMING`, `SIM_CONGESTED_CARS`, `SIM_NON_CONGESTED_CARS`
- **Arduino / controller**
  - `ARDUINO_PORT`, `ARDUINO_BAUDRATE`, `SIMULATE_SIGNALS_WHEN_NO_ARDUINO`
  - `AUTO_MODE_DEFAULT` (start in automatic lane cycling)
- **Detection & tracking**
  - Vehicle size, color, and motion thresholds (`MIN_VEHICLE_AREA`, `MAX_VEHICLE_AREA`, `CAR_COLOR_RANGES`, etc.)
  - Stability and smoothing parameters for tracks and collisions
- **Violation detection (E.Y.E.)**
  - `RED_LIGHT_SPEED_THRESHOLD`, `YELLOW_ABUSE_SPEED_THRESHOLD`, `YELLOW_SAFE_DISTANCE`
  - `ENABLE_ILLEGAL_TURN`, `TURN_ANGLE_THRESHOLD`, `ILLEGAL_TURN_FRAMES`
- **License plate recognition**
  - `ENABLE_LICENSE_PLATE_DETECTION`, ROI and aspect‑ratio thresholds
  - OCR cadence and throttling: `LP_DETECT_EVERY_N_FRAMES`, `LP_MAX_VEHICLES_PER_FRAME`, `LP_PER_VEHICLE_COOLDOWN_SECONDS`
  - Tesseract configuration: `TESSERACT_CMD`, `TESSERACT_CONFIG`
- **Logging and analytics**
  - `LOG_DIRECTORY` (defaults to `USAD-Model/logs`)
  - Filenames for `EVENT_LOG_FILE`, `VIOLATION_LOG_FILE`, `ACCIDENT_LOG_FILE`
  - Analytics windows and thresholds.

## 🎮 Running the System

From the `USAD` root (with the virtual environment activated):

```bash
cd USAD-Model
python main.py
```

If the camera and configuration are valid, a window titled “USAD - AI Traffic Management System” will open and start processing frames.

### OCR smoke test (optional)

To quickly verify that license‑plate OCR works and can read synthetic test plates:

```bash
cd USAD-Model
python ocr_smoke_test.py
```

The script prints pass/fail status and confidence for several synthetic plate images.

### Keyboard controls

While the main window is focused:

- `Q` or `ESC` – Quit the application
- `R` – Reset the system (clears tracked vehicles and active events)
- `A` – Switch to automatic lane‑cycling mode
- `S` – Print statistics and generate an analytics report

## 📊 Data, Logs, and Analytics

### Runtime logs

At runtime, CSV logs are written to `USAD-Model/logs/` (the directory is created if it does not exist):

- `traffic_events.csv` – All high‑level events (violations, accidents, notifications) with timestamps and descriptions.
- `violations.csv` – Detailed per‑violation records: lane, type, speed, license plate, and coordinates.
- `accidents.csv` – Confirmed accidents with duration, lane, involved vehicles, and whether an emergency notification was issued.
- `license_plates.csv` – Successfully recognized license plates along with detection confidence and location.

### Analytics reports

When you press `S` during operation, the system computes aggregated statistics and writes a human‑readable report to:

- `USAD-Model/logs/analytics_report.txt`

The report includes, for the configured time window:

- Total violations by type and by lane
- High‑risk lanes (those with ≥3× the average violation count)
- Peak violation hours
- Accident counts and durations
- Emergency‑notification counts

## 🔧 Arduino Traffic Controller

### Pin configuration

The default sketch expects the following LED wiring:

```
Lane 1 (North): Green = 2,  Yellow = 3,  Red = 4
Lane 2 (South): Green = 5,  Yellow = 6,  Red = 7
Lane 3 (East):  Green = 8,  Yellow = 9,  Red = 10
Lane 4 (West):  Green = 11, Yellow = 12, Red = 13
```

Update `arduino/traffic_controller.ino` if you wire your hardware differently.

### Serial protocol

The Python controller sends simple text commands over serial:

- `LANE1` – Activate Lane 1 (North)
- `LANE2` – Activate Lane 2 (South)
- `LANE3` – Activate Lane 3 (East)
- `LANE4` – Activate Lane 4 (West)
- `AUTO` – Return to automatic lane‑cycling mode

The sketch is responsible for translating these into LED sequences (green → yellow → red) using the configured timing.

### Timing and adaptive control

Signal timing is driven by parameters in `config.py`:

- `GREEN_TIME` – Base green duration per lane (default 25 seconds)
- `YELLOW_TIME` – Yellow duration (default 4 seconds)
- `MIN_GREEN_TIME`, `MAX_GREEN_TIME` – Bounds for adaptive adjustments

The adaptive controller uses per‑lane vehicle counts to extend green time for congested lanes and shorten it for lightly loaded lanes, staying within these bounds while ensuring fair rotation.

If the Arduino is disconnected or cannot be opened on `ARDUINO_PORT`, USAD automatically enters **simulation mode** and maintains an internal model of signal states without driving hardware.

## 🎯 E.Y.E. Violation Detection

The **Eyeing Your Encounter (E.Y.E.)** subsystem is implemented in `USAD-Model/violation_detector.py` and uses lane geometry, vehicle trajectories, and current signal states to flag unsafe behavior.

### Red‑light violations

- Each lane has a configured stop line in `config.LANES`.
- When the signal for a lane is **RED**, the system checks whether a vehicle’s tracked path crosses the stop line.
- A minimum speed threshold (`RED_LIGHT_SPEED_THRESHOLD`) prevents slow, creeping movements from being misclassified.

## 🚨 Emergency Notification System

The emergency‑notification pipeline, implemented in `USAD-Model/emergency_notifier.py`, is triggered only for **confirmed** accidents.

When an accident is confirmed:

1. **SMS simulation** – A formatted text block is printed to the console with timestamp, hotline, lane, coordinates, and involved vehicle IDs.
2. **Call simulation** – A second block simulates placing a phone call to `EMERGENCY_HOTLINE` (default `911`), summarizing the incident.
3. **Event logging** – The accident is recorded to `accidents.csv` and to the general `traffic_events.csv` log.
4. **Cooldown enforcement** – `NOTIFICATION_COOLDOWN` (default 60 seconds) prevents repeated notifications for the same accident.

## 🎨 Visual Interface

The main OpenCV window renders a composite overlay on top of the camera feed:

- **Vehicle tracking** – Bounding boxes labeled with stable IDs and inferred vehicle type.
- **Lane regions and stop lines** – Color‑coded lane polygons and yellow stop lines for each approach.
- **Accident markers** – Highlighted overlays for detected stopped vehicles and collisions.
- **Violation markers** – Annotation for red‑light.
- **License plates** – Bounding boxes and text labels for recognized plates when OCR is enabled.
- **Status panel** – Per‑lane counts, active lane and phase, FPS, and summary counters.

## 📝 System Behavior

### Adaptive traffic control

- **Congestion‑aware green times** – Lanes with more vehicles receive longer greens; lightly loaded lanes use shorter phases within `MIN_GREEN_TIME` and `MAX_GREEN_TIME`.
- **Accident resilience** – Accident detection can influence timing (e.g., extended clearance windows) depending on configuration.
- **Fairness** – Round‑robin lane selection ensures that no lane starves even under asymmetrical demand.

## 🐛 Troubleshooting

### Python packages not found (e.g., `ModuleNotFoundError: No module named 'cv2'`)

This usually indicates that dependencies were installed into a different Python environment than the one you are using.

1. Ensure you are in the `USAD` root directory.
2. Activate the virtual environment:
   - Windows: `./venv/Scripts/Activate.ps1`
   - Linux/macOS: `source venv/bin/activate`
3. Reinstall dependencies:

   ```bash
   pip install -r requirements.txt
   ```

4. On Windows, if `python main.py` still fails, run explicitly with the venv Python:

   ```bash
   cd USAD-Model
   ../venv/Scripts/python.exe main.py
   ```

### Arduino not connecting

- Confirm `ARDUINO_PORT` in `USAD-Model/config.py` matches your actual COM port (e.g., `COM5`).
- Ensure no other application is using the same serial port.
- Try a different USB cable or port.
- If connection fails, USAD automatically runs in **simulation mode**; hardware signals will not change, but the software model and analytics still function.

### Camera not opening

- Verify `CAMERA_SOURCE` in `USAD-Model/config.py` (e.g., 0, 1, 2, or a video file path).
- Close any other application that may be using the same camera.
- Try alternate capture backends or another camera index if your system has multiple devices.

### Weak or noisy vehicle detection

- Adjust `BACKGROUND_THRESHOLD` and related foreground‑mask parameters in `config.py`.
- Ensure the scene has adequate, stable lighting.
- Refine lane regions and intersection polygons so that they match your camera view.
- Tune `MIN_VEHICLE_AREA`, `MAX_VEHICLE_AREA`, and the color‑range settings for your specific vehicles.

### License plates not detected or low confidence

- Confirm EasyOCR is installed (via `pip install easyocr`) and that Tesseract, if used, is installed and accessible.
- Tune plate size and aspect‑ratio limits (`LP_MIN_WIDTH`, `LP_MAX_WIDTH`, `LP_MIN_HEIGHT`, `LP_MAX_HEIGHT`).
- Verify `TESSERACT_CMD` and `TESSERACT_CONFIG` if relying on Tesseract.
- Use higher‑resolution video or adjust camera placement so plates are clearer and less oblique.

## 📄 License

This project is open source and distributed under the MIT License.

## 👥 Contributors

USAD‑AI‑TRAFFIC‑SYSTEM Team

## 🤝 Contributing

Contributions are welcome. Please open an issue to discuss significant changes before submitting a pull request.

## 📧 Contact

For questions, bug reports, or feature requests, please open an issue on GitHub.

---

**USAD** – Making intersections safer and smarter with AI 🚦✨
