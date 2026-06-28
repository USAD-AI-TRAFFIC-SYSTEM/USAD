# USAD - Urban Smart Adaptive Dispatcher

An AI-powered traffic management system that uses computer vision to enhance road safety and optimize intersection flow through intelligent, data-driven control.


<img width="1909" height="1080" alt="638378206_1523991645367022_5662007556454183657_n" src="https://github.com/user-attachments/assets/e8dbea4a-ae4f-420a-9abc-cd48f0949bcb" />

## 🚦 Features

### Core Capabilities
- **Real-time Vehicle Detection**: Uses OpenCV background subtraction and contour detection to track vehicles at intersections
- **Accident Detection**: Automatically detects stopped or collided vehicles
- **Emergency Notifications**: Simulates SMS and call notifications to hotlines (#911) during accidents
- **E.Y.E. (Eyeing Your Encounter)**: Detects unsafe and illegal vehicle behaviors including:
  - Red-light violations
  - Yellow-light abuse
  - Illegal turns
- **License Plate Recognition**: Detects and reads license plates using OCR for vehicle tracking
- **Adaptive Traffic Control**: Dynamically adjusts traffic light timing based on congestion and accidents
- **Arduino Integration**: Controls physical traffic lights via serial communication (4)
- **Analytics & Logging**: Comprehensive event logging and analytics for traffic patterns

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        USAD System                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌───────────┐    ┌─────────────┐    ┌────────────────┐     │
│  │  Camera   │───▶│   Vehicle   │───▶│   Accident     │    │
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
│                    │  (Arduino COM4 Integration)     │      │
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
- **Camera**: Webcam or CCTV with bird's-eye view of intersection
- **Arduino**: Connected to COM4 with traffic light controller
  - 4 lanes with Red, Yellow, Green LEDs each
  - Pin configuration as per `arduino/traffic_controller.ino`
- **Computer**: Windows PC with USB ports

### Software
- Python 3.8 or higher
- Arduino IDE (for uploading traffic controller sketch)
- Tesseract OCR (for license plate detection)

## 🚀 Installation

### 1. Clone the Repository
```bash
git clone https://github.com/USAD-AI-TRAFFIC-SYSTEM/USAD--AI-TRAFFIC-SYSTEM.git
cd USAD
```

### 2. Install Python Dependencies

**⚠️ IMPORTANT: Make sure you're in the USAD root directory (not USAD-Model) throughout this process**

#### Windows (PowerShell/Command Prompt):
```bash
# Create virtual environment in the USAD root directory
python -m venv venv

# Activate the virtual environment
.\venv\Scripts\Activate.ps1

# Install dependencies from the root directory
pip install -r requirements.txt

# Run the application
cd USAD-Model
..\venv\Scripts\python.exe main.py
```

#### Linux/Mac:
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
cd USAD-Model
python main.py
```

### 3. Install Tesseract OCR (Optional for License Plates)
Download and install from: https://github.com/tesseract-ocr/tesseract

After installation, update the path in `license_plate_detector.py` if needed:
```python
pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
```

### 4. Upload Arduino Sketch
1. Open `arduino/traffic_controller.ino` in Arduino IDE
2. Connect Arduino to COM4
3. Upload the sketch to the Arduino

### 5. Configure Camera
Edit `config.py` to set your camera source:
```python
CAMERA_SOURCE = 0  # 0 for webcam, or path to video file
```

## 🎮 Usage

### Running the System
```bash
cd USAD-Model
python main.py
```

### Keyboard Controls
- **Q** or **ESC**: Quit application
- **R**: Reset system (clear all tracked vehicles and events)
- **A**: Switch to automatic cycling mode
- **1-4**: Manually activate specific lanes (Lane 1-4)
- **S**: Print statistics and generate analytics report

### Configuration
Edit `config.py` to customize:
- Lane regions and stop lines
- Detection thresholds
- Timing parameters
- Arduino port settings
- Camera settings
- Feature toggles

## 📊 Output & Analytics

### Logged Data
All events are logged to CSV files in the `logs/` directory:
- `traffic_events.csv`: All traffic events
- `violations.csv`: Detailed violation records
- `accidents.csv`: Accident records with emergency notifications

### Analytics Reports
Press **S** during operation or check `logs/analytics_report.txt` for:
- Total violations by type and lane
- High-risk intersections (3x+ violations)
- Peak violation hours
- Accident statistics
- Emergency notification count

### Sample Analytics Output
```
======================================================================
USAD TRAFFIC ANALYTICS REPORT
Generated: 2026-01-22 14:30:00
======================================================================

VIOLATION STATISTICS
----------------------------------------------------------------------
Total Violations: 47

By Type:
  - RED_LIGHT_VIOLATION: 18
  - YELLOW_ABUSE: 12
  - ILLEGAL_TURN: 17

High-Risk Lanes (3x+ violations):
  - LANE3: 21 violations (3.5x average)

Peak Violation Hours:
  - 17:00: 15 violations
  - 8:00: 12 violations
```

## 🔧 Arduino Traffic Controller

### Pin Configuration
```
Lane 1 (North): Green=2,  Yellow=3,  Red=4
Lane 2 (South): Green=5,  Yellow=6,  Red=7
Lane 3 (East):  Green=8,  Yellow=9,  Red=10
Lane 4 (West):  Green=11, Yellow=12, Red=13
```

### Serial Commands
The Python system sends these commands to Arduino:
- `LANE1`: Activate Lane 1 (North)
- `LANE2`: Activate Lane 2 (South)
- `LANE3`: Activate Lane 3 (East)
- `LANE4`: Activate Lane 4 (West)
- `AUTO`: Return to automatic cycling mode

### Timing
- Green Light: 5 seconds
- Yellow Light: 3 seconds
- Red Light: Automatic (when other lanes are active)

## 🎯 E.Y.E. Violation Detection

The **Eyeing Your Encounter (E.Y.E.)** system uses classical computer vision (no machine learning) to detect:

### Red Light Violations
- Detects vehicles crossing stop line while signal is red
- Requires minimum speed threshold to avoid false positives
- Tracks vehicle trajectory across stop line

### Yellow Light Abuse
- Detects vehicles speeding through yellow lights
- Calculates if vehicle could have safely stopped
- Uses speed and distance thresholds

### Illegal Turns
- Analyzes vehicle direction vs. lane direction
- Detects turns exceeding angle threshold
- Confirms over multiple frames

## 🚨 Emergency Notification System

When an accident is detected and confirmed:
1. **SMS Simulation**: Displays formatted emergency SMS
2. **Call Simulation**: Simulates emergency hotline call (#911)
3. **Event Logging**: Records notification in accident log
4. **Cooldown**: Prevents spam notifications (60-second cooldown)

### Sample Emergency Notification
```
╔══════════════════════════════════════════════════════════════╗
║             EMERGENCY ACCIDENT NOTIFICATION                  ║
╠══════════════════════════════════════════════════════════════╣
║ Time: 2026-01-22 14:25:30                                    ║
║ Hotline: 911                                                 ║
║                                                              ║
║ ACCIDENT DETAILS:                                            ║
║ - Type: COLLISION                                            ║
║ - Location: North Lane                                       ║
║ - Coordinates: (640, 360)                                    ║
║ - Vehicles Involved: 2                                       ║
║ - Vehicle IDs: 123, 456                                      ║
║                                                              ║
║ IMMEDIATE RESPONSE REQUIRED                                  ║
╚══════════════════════════════════════════════════════════════╝
```

## 🎨 Visual Interface

The system displays real-time video with overlays:
- **Vehicle Tracking**: Bounding boxes with IDs and types
- **Lane Regions**: Color-coded lane boundaries
- **Stop Lines**: Yellow lines marking traffic signal positions
- **Accidents**: Red cross markers with alerts
- **Violations**: Orange star markers with labels
- **License Plates**: Yellow boxes with OCR text
- **Status Panel**: System stats, FPS, counts, controls

## 📝 System Behavior

### Adaptive Traffic Control
- **Congestion Detection**: Extends green time for lanes with 5+ vehicles
- **Accident Priority**: Gives extra time to accident lanes for clearance
- **Fair Distribution**: Maintains balanced timing across all lanes

### Vehicle Classification
Based on contour area:
- **SMALL**: 800-2500 px² (Motorcycles, small cars)
- **MEDIUM**: 2500-6000 px² (Sedans, SUVs)
- **LARGE**: 6000-15000 px² (Trucks, buses)

## 🐛 Troubleshooting

### ModuleNotFoundError: No module named 'cv2' (or other packages)
This occurs when packages are installed in the system Python but not in the virtual environment.

**Solution:**
1. Make sure you're in the USAD root directory (not USAD-Model)
2. Ensure your virtual environment is activated:
   - Windows: `.\venv\Scripts\Activate.ps1`
   - Linux/Mac: `source venv/bin/activate`
3. Reinstall dependencies in the venv:
   ```bash
   pip install -r requirements.txt
   ```
4. On Windows, if `python main.py` still fails, explicitly use the venv's Python:
   ```bash
   cd USAD-Model
   ..\venv\Scripts\python.exe main.py
   ```

### Arduino Not Connecting
- Verify Arduino is connected to COM4
- Check if port is in use by another application
- Try different USB port
- System will run in simulation mode if Arduino unavailable

### Camera Not Opening
- Verify `CAMERA_SOURCE` in `config.py`
- Try different camera index (0, 1, 2, etc.)
- Check camera permissions

### Poor Vehicle Detection
- Adjust `BACKGROUND_THRESHOLD` in `config.py`
- Ensure good lighting conditions
- Calibrate lane regions for your camera angle
- Modify `MIN_VEHICLE_AREA` and `MAX_VEHICLE_AREA`

### License Plate Not Detected
- Ensure Tesseract is installed correctly
- Adjust plate size thresholds in `config.py`
- Check `TESSERACT_CONFIG` string
- Works best with clear, frontal vehicle views

## 📄 License

This project is open-source and available under the MIT License.

## 👥 Contributors

USAD-AI-TRAFFIC-SYSTEM Team

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues.

## 📧 Contact

For questions or support, please open an issue on GitHub.

---

**USAD** - Making intersections safer and smarter with AI 🚦✨
