"""
Build USAD as a standalone Windows executable using PyInstaller.
Run this script to create the USAD.exe file.
"""

import PyInstaller.__main__
import os
import shutil
from pathlib import Path

# Paths
project_root = Path(__file__).parent
usad_model = project_root / "USAD-Model"
icon_file = project_root / "Logo 1.ico"
dist_folder = project_root / "dist"

print("=" * 60)
print("USAD APP BUILDER")
print("=" * 60)

# Check if icon exists
if not icon_file.exists():
    print(f"⚠️  Icon file not found: {icon_file}")
    print("Using default icon instead")
    icon_arg = None
else:
    icon_arg = str(icon_file)
    print(f"✓ Using icon: {icon_file}")

# PyInstaller arguments  (Windows uses ; as --add-data separator)
pyinstaller_args = [
    str(usad_model / "app.py"),           # Main app file
    "--name=USAD",                         # Executable name
    "--onefile",                           # Single executable file
    "--console",                           # Keep console for now to see errors (change to --windowed later)
    f"--icon={icon_arg}" if icon_arg else "",  # App icon
    
    # Add local USAD-Model modules so PyInstaller can find them
    f"--paths={usad_model}",
    
    # Add data files (assets, fonts, etc.)  — use ; on Windows
    f"--add-data={usad_model / 'assets'};assets",
    f"--add-data={usad_model / 'logs'};logs",
    
    # Hidden imports — local modules
    "--hidden-import=main",
    "--hidden-import=config",
    "--hidden-import=vehicle_detector",
    "--hidden-import=accident_detector",
    "--hidden-import=violation_detector",
    "--hidden-import=license_plate_detector",
    "--hidden-import=traffic_controller",
    "--hidden-import=emergency_notifier",
    "--hidden-import=event_logger",
    "--hidden-import=dashboard",
    
    # Hidden imports — third party packages
    "--hidden-import=cv2",
    "--hidden-import=numpy",
    "--hidden-import=customtkinter",
    "--hidden-import=PIL",
    "--hidden-import=PIL.Image",
    "--hidden-import=PIL.ImageTk",
    "--hidden-import=PIL._tkinter_finder",
    "--hidden-import=easyocr",
    "--hidden-import=serial",
    "--hidden-import=serial.tools",
    "--hidden-import=serial.tools.list_ports",
    
    # Collect all package data (themes, assets, models)
    "--collect-all=customtkinter",
    "--collect-all=easyocr",
    "--collect-all=PIL",
    
    # Ensure tkinter is bundled
    "--collect-all=tkinter",
    
    # Output directory
    f"--distpath={dist_folder}",
    f"--workpath={project_root / 'build'}",
    f"--specpath={project_root / 'build'}",
    
    # Overwrite previous build
    "--noconfirm",
]

# Remove empty strings (e.g. when no icon)
pyinstaller_args = [arg for arg in pyinstaller_args if arg]

print("\nBuilding executable...")
print("-" * 60)

try:
    PyInstaller.__main__.run(pyinstaller_args)
    print("\n" + "=" * 60)
    print("✓ BUILD SUCCESSFUL!")
    print("=" * 60)
    print(f"\nYour executable is ready at:")
    print(f"  {dist_folder / 'USAD.exe'}")
    print(f"\nYou can now:")
    print(f"  1. Double-click USAD.exe to run the app")
    print(f"  2. Create a shortcut on your desktop")
    print(f"  3. Distribute USAD.exe to users")
    print(f"\n💡 TIP: Once the app works, change --console to --windowed")
    print(f"   in build_app.py to hide the terminal window.")
    
except Exception as e:
    print(f"\n❌ BUILD FAILED: {e}")
    exit(1)
