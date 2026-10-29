# Distributing USAD to Non-Technical Users

## Quick Setup (For You)

### Step 1: Build the Executable
On your development machine:

1. Open Command Prompt (or PowerShell)
2. Navigate to the USAD root folder
3. Double-click **SETUP.bat**
4. Wait for the build to complete (10-20 minutes; the first build also downloads the EasyOCR models)
5. You'll see `dist/USAD.exe` created

### Step 2: Create Desktop Shortcut
Right-click on `dist/USAD.exe` → Send to → Desktop (create shortcut)

### Step 3: Distribute to Users
Zip the `dist` folder (or give users just **USAD.exe**). Include:
- `USAD.exe` (required)
- `lane_config.json` / `detection_config.json` if you calibrated inside the EXE and the camera/board stay the same
- `camera_config.json` only if the other PC numbers its cameras the same way (otherwise reassign in Camera Setup)
- `logs/` only to carry over analytics history

---

## For Users (Non-Technical)

### Installation
1. Copy `USAD.exe` (or unzip the folder) to a normal folder such as Desktop or Documents.
   Not `Program Files`: USAD saves its settings and logs next to the EXE.
2. Right-click on it and select "Create Shortcut"
3. Drag the shortcut to your Desktop for easy access

### Running USAD
Simply **double-click USAD.exe** or the shortcut

---

## What Was Included in the Executable

The USAD.exe includes:
- ✓ All Python dependencies (OpenCV, NumPy, PySerial, EasyOCR, CustomTkinter, etc.)
- ✓ EasyOCR license plate models (no download on first start)
- ✓ The dashboard (React UI) and current lane / traffic light calibration
- ✓ Application icon (USAD.ico)
- ✓ Font assets (Poppins)
- ✓ Logs directory
- ✓ No Python installation required on user's computer
- ✓ No virtual environment needed
- ✓ Runs immediately when double-clicked

---

## File Size

The executable is about 340 MB: Python libraries (mostly PyTorch for EasyOCR), the dashboard, and the EasyOCR models. The first launch takes longer (often 30-60 s) while it unpacks.

---

## Troubleshooting

### "USAD.exe Not Running"
- Check if Windows Defender is blocking it 
  - If blocked, click "More info" → "Run anyway"
- Try right-clicking → Run as Administrator

### "Missing Camera or Arduino"
- Check if device drivers are installed
- Verify camera is connected and functional
- The Arduino's COM port is detected automatically; clone boards (CH340 chip) may need their USB driver

### "Window is blank or does not open"
The app window uses Microsoft Edge WebView2 Runtime (built into Windows 11 and updated Windows 10).
Install it from Microsoft, or open http://127.0.0.1:8000 in a browser while USAD is running.

### "No Internet Required"
Everything, including the EasyOCR models, is inside USAD.exe, so it runs fully offline, even on first start.

---

## Building Updates

When you make changes to the code:
1. Edit files in `USAD-Model/`
2. Run **SETUP.bat** again
3. Share the new **USAD.exe**

Close USAD first (a running USAD.exe cannot be replaced). Users just need to replace the old
executable with the new one; their `*_config.json` files and `logs/` next to it are kept.
