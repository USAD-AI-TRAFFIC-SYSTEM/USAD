# Distributing USAD to Non-Technical Users

## Quick Setup (For You)

### Step 1: Build the Executable
On your development machine:

1. Open Command Prompt (or PowerShell)
2. Navigate to the USAD root folder
3. Double-click **SETUP.bat**
4. Wait for the build to complete (2-5 minutes)
5. You'll see `dist/USAD.exe` created

### Step 2: Create Desktop Shortcut
Right-click on `dist/USAD.exe` → Send to → Desktop (create shortcut)

### Step 3: Distribute to Users
You can give users just the **USAD.exe** file, and they can:
- Double-click it to run
- Drag to create a shortcut
- Pin it to Start Menu

---

## For Users (Non-Technical)

### Installation
1. Download or copy `USAD.exe` to your computer
2. Right-click on it and select "Create Shortcut"
3. Drag the shortcut to your Desktop for easy access

### Running USAD
Simply **double-click USAD.exe** or the shortcut

---

## What Was Included in the Executable

The USAD.exe includes:
- ✓ All Python dependencies (OpenCV, NumPy, PySerial, EasyOCR, CustomTkinter, etc.)
- ✓ Application icon (USAD.ico)
- ✓ Font assets (Poppins)
- ✓ Logs directory
- ✓ No Python installation required on user's computer
- ✓ No virtual environment needed
- ✓ Runs immediately when double-clicked

---

## File Size

The executable is typically 100-150 MB, which includes all Python libraries and dependencies bundled together.

---

## Troubleshooting

### "USAD.exe Not Running"
- Check if Windows Defender is blocking it 
  - If blocked, click "More info" → "Run anyway"
- Try right-clicking → Run as Administrator

### "Missing Camera or Arduino"
- Check if device drivers are installed
- Verify camera is connected and functional
- Arduino COM5 must be available (check Device Manager)

### "No Internet Required"
The app works offline after initial download. No internet connection needed to run.

---

## Building Updates

When you make changes to the code:
1. Edit files in `USAD-Model/`
2. Run **SETUP.bat** again
3. Share the new **USAD.exe**

Users just need to replace the old executable with the new one.
