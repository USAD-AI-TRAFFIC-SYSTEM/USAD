# USAD - Urban Smart Adaptive Dispatcher

An adaptive traffic light signal system using classical computer vision to enhance road safety and optimize traffic flow at intersections.

## Features

- **Adaptive Signal Timing**: Dynamically adjusts green light duration based on vehicle counts
- **Congestion Detection**: Identifies and responds to traffic congestion
- **E.Y.E. Module**: Detects traffic violations (red-light running, yellow-light abuse, illegal turns)
- **Emergency Response**: Sends notifications on collision detection
- **License Plate Detection**: Optional OCR for vehicle reporting
- **Arduino Integration**: Hardware control via serial communication
- **Comprehensive Logging**: Event and violation tracking

## Project Structure

```
USAD/
├── src/
│   ├── vision/              # Video processing & background subtraction
│   ├── detection/           # Vehicle detection & tracking
│   ├── signal_control/      # Signal timing logic
│   ├── eye_module/          # Violation detection module
│   ├── notification/        # SMS/Email alerts
│   ├── ocr/                 # License plate recognition
│   └── logging/             # Event logging system
├── arduino/                 # Arduino traffic signal controller
├── config/                  # Configuration files
├── logs/                    # Generated log files
├── tests/                   # Unit tests
└── docs/                    # Documentation
```

## Installation

1. Clone or download the repository
2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Quick Start (from scratch)

1. **Clone the repo**
   ```bash
   git clone https://github.com/USAD-AI-TRAFFIC-SYSTEM/USAD--AI-TRAFFIC-SYSTEM.git
   cd USAD--AI-TRAFFIC-SYSTEM
   ```

2. **Create a virtual environment (Windows PowerShell)**
   ```powershell
   python -m venv venv
   .\venv\Scripts\Activate.ps1
   ```

3. **Upgrade pip (recommended)**
   ```powershell
   python -m pip install --upgrade pip
   ```

4. **Install requirements**
   ```powershell
   pip install -r requirements.txt
   ```

5. **(Optional) Install Tesseract OCR** for license plate detection
   - Windows: Install from https://github.com/UB-Mannheim/tesseract/wiki and ensure the binary is on PATH
   - Linux: `sudo apt-get install tesseract-ocr`

6. **Run the app**
   ```powershell
   python main.py
   ```

7. **Arduino upload**
   - Open `arduino/traffic_signal_controller.ino` in Arduino IDE
   - Select your board and port
   - Upload

3. Install Tesseract OCR (if using license plate detection):
   - **Windows**: Download installer from [GitHub Tesseract](https://github.com/UB-Mannheim/tesseract/wiki)
   - **Linux**: `sudo apt-get install tesseract-ocr`

4. Upload Arduino code to your Arduino board using Arduino IDE

## Configuration

Edit `config/config.yaml` to customize:
- Video source (webcam or file)
- Lane regions (ROI coordinates)
- Signal timing parameters
- Notification settings
- Arduino serial port

## Usage

```bash
python main.py
```

## Hardware Requirements

- Arduino Uno/Mega
- 4 traffic light sets (12 digital pins)
- 1 emergency light (1-2 pins)
- Optional: buzzer, relay modules
- Webcam or CCTV camera

## Arduino Pin Configuration

- North: RED=2, YELLOW=3, GREEN=4
- South: RED=5, YELLOW=6, GREEN=7
- East: RED=8, YELLOW=9, GREEN=10
- West: RED=11, YELLOW=12, GREEN=13
- Emergency: A0
- Buzzer: A1

## Serial Protocol (Python ↔ Arduino)

Commands: `DIRECTION,GREEN_TIME,YELLOW_TIME,RED_TIME`

Examples:
- `N,30,5,40` - North: 30s green, 5s yellow, 40s red
- `EMERGENCY` - Activate emergency mode
- `CLEAR_EMERGENCY` - Clear emergency mode
- `STATUS` - Request current status

## License

© 2026 USAD Development Team

## Support

For issues or questions, refer to documentation in `docs/` folder.
