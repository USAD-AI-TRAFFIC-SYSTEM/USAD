# USAD System Functions and Features

This document explains what each main file in the USAD system does, what its most important functions are, and gives easy-to-understand examples.

USAD stands for **Urban Smart Adaptive Detection**. It watches an intersection with a camera, looks for cars, spots red‑light violations and accident‑like situations, and can drive a small traffic‑light setup using an Arduino.

---

## 1. `main.py` – Core Brain of the System

**What this file does**  
`main.py` is the heart of USAD. It:
- Opens the camera
- Reads video frames
- Finds and follows cars
- Checks for red‑light violations and accidents
- Talks to the Arduino traffic lights
- Sends messages to the logging and emergency modules

### Important pieces

- **`class USAD`**  
  This class is the main controller of the whole system.

  Main jobs:
  - Start the camera and Arduino
  - Run the main loop that processes each video frame
  - Ask other modules to detect cars, violations, and accidents
  - Decide which lane should have green/yellow/red

  ```python
  from main import USAD

  # Create the main USAD engine
  usad = USAD()

  # Set up hardware
  usad.initialize_camera()
  usad.initialize_arduino()

  # Example of processing a single frame
  ret, frame = usad.cap.read()
  if ret:
      processed_frame = usad.process_frame(frame)
  ```

- **`initialize_camera()`**  
  Tries different ways to open the camera, sets the image size and frame‑rate, and warms up the feed.

  ```python
  success = usad.initialize_camera()
  if not success:
      print("Camera could not be opened. Check CAMERA_SOURCE in config.py.")
  ```

- **`initialize_arduino()`**  
  Tries to connect to the Arduino on the configured COM port. If it fails, the system switches to **simulation mode**, where it only pretends to control the lights.

  ```python
  # Try to connect to the Arduino
  usad.initialize_arduino()
  # If it cannot connect, USAD prints a message and continues in simulation mode.
  ```

- **`process_frame(frame)`**  
  This is the most important function. For each frame:
  1. **Find cars** – calls `vehicle_detector.detect_vehicles()`.
  2. **Update signals** – uses lane counts to decide which lane should be green or yellow.
  3. **Check for accidents** – calls `accident_detector.detect_accidents()`.
  4. **Send emergency alerts** for confirmed accidents.
  5. **Check for red‑light violations** – calls `violation_detector.detect_violations()`.
  6. **Log events** – tells `EventLogger` to store violations and accidents.
  7. **Try license‑plate reading** – if enabled.
  8. **Draw overlays** – draws boxes and labels on the frame so the user can see what is happening.

  ```python
  # Inside a loop
  ret, frame = usad.cap.read()
  if ret:
      processed = usad.process_frame(frame)
      # "processed" now has drawings for cars, violations, and accidents.
  ```

---

## 2. `app.py` – Main Window and Live View

**What this file does**  
`app.py` creates a modern desktop window using **CustomTkinter**. It shows:
- The live camera view with all overlays
- A side panel with lane states, timers, and counts
- Buttons and keyboard shortcuts to control the system

### Important pieces

- **`class USADApp(ctk.CTk)`**  
  Main graphical application window.

  Main jobs:
  - Create the window and set icons
  - Create the `USAD` engine from `main.py`
  - Show the camera view inside the window
  - Show a right‑hand panel with status info (per‑lane lights, counts, FPS)
  - Forward key presses (Q, R, A, 1–4, S) to the USAD engine

  ```python
  import customtkinter as ctk
  from app import USADApp

  if __name__ == "__main__":
      ctk.set_appearance_mode("dark")
      app = USADApp()
      app.mainloop()
  ```

- **`_tick()`**  
  This function runs over and over (many times per second). It:
  - Reads a frame from the camera
  - Sends it to `usad.process_frame()`
  - Converts the processed frame into an image for the window
  - Updates the lane lights, lane counts, and FPS labels

  ```python
  def _tick(self):
      ret, frame = self.usad.cap.read()
      if ret:
          processed = self.usad.process_frame(frame)
          self._update_canvas(processed)
          self._update_side_panel()
      # Call _tick again after 10 ms
      self.after(10, self._tick)
  ```

- **Dashboard tab**  
  The "Dashboard" tab button starts `dashboard.py` as a separate window. That window shows past data (not the live camera).

---

## 3. `dashboard.py` – Analytics View (No Camera)

**What this file does**  
`dashboard.py` is a separate analytics dashboard. It does **not** use the camera. Instead, it reads the CSV log files and shows:
- Total number of events, violations, and accidents
- Tables listing each violation, accident, and event
- Simple charts and summaries

### Important pieces

- **`DashboardApp` class**  
  Creates a full‑screen window with:
  - A top bar showing totals (how many violations, accidents, events)
  - Tabs: **Overview**, **Violations**, **Accidents**, **Traffic Events**

  ```python
  import customtkinter as ctk
  from dashboard import DashboardApp

  if __name__ == "__main__":
      ctk.set_appearance_mode("dark")
      app = DashboardApp()
      app.mainloop()
  ```

- **`load_csv(name)`**  
  A helper that finds and reads CSV files like `violations.csv` or `accidents.csv`.

  ```python
  from dashboard import load_csv

  violations_df = load_csv("violations.csv")
  accidents_df = load_csv("accidents.csv")
  ```

  *Example:*  
  When you click the "Violations" tab, the app shows a table with each red‑light violation, its time, which lane it happened in, and the speed.

---

## 4. `vehicle_detector.py` – Finding and Following Cars

**What this file does**  
`vehicle_detector.py` looks at each frame and decides **where the cars are** and **where they move** over time.

### Important pieces

- **`class Vehicle`**  
  Represents a single car the system is tracking.

  It stores:
  - A unique ID number (e.g., Car #3)
  - The car’s position in the image
  - Recent movement history (to estimate direction and speed)

  It also has helper methods such as:
  - `get_speed()` – how fast the car is moving across the image
  - `get_direction()` – whether it is going roughly north‑south or east‑west

  ```python
  from vehicle_detector import Vehicle

  # Example of creating a Vehicle object
  v = Vehicle(vehicle_id=1, bbox=(100, 200, 160, 260))
  speed = v.get_speed()
  direction = v.get_direction()
  ```

- **Vehicle tracking logic**  
  The detector:
  - Finds moving spots in the frame
  - Filters out noise (small blobs, shadows, etc.)
  - Either creates a new `Vehicle` or updates an existing one
  - Keeps track of which lane each car is in

  *Example:*  
  A car enters Lane 2 from the bottom of the image. The detector creates `Vehicle #5`, follows it for several frames, and later knows that "Vehicle #5 is in South lane and moving upwards".

---

## 5. `accident_detector.py` – Spotting Accident‑Like Events

**What this file does**  
`accident_detector.py` looks at the list of tracked cars and tries to detect **accident‑like situations**, such as:
- Cars that have collided or are extremely close together
- Cars that stopped suddenly and stayed stopped in a risky area

### Important pieces

- **`class Accident`**  
  Represents a single accident event.

  It stores:
  - A unique accident ID
  - Type (for example: collision)
  - Which cars are involved
  - Where it is in the intersection
  - How long it has been going on
  - Whether it has already triggered an emergency notification

  ```python
  from accident_detector import Accident

  acc = Accident(accident_id=1, accident_type="COLLISION", vehicles_involved=[1, 2])
  description = acc.get_description()
  ```

- **`detect_accidents(vehicles, frame)`**  
  Checks distances and overlap between cars. When two cars are too close for long enough (and other rules match), it marks an accident and keeps track of it across frames.

  ```python
  from accident_detector import AccidentDetector

  detector = AccidentDetector()
  accidents = detector.detect_accidents(vehicles=current_vehicles, frame=frame)

  for acc in accidents:
      print("Detected accident:", acc.get_description())
  ```

  *Example:*  
  Two cars overlap in the image and stay locked together instead of passing by. The detector creates an `Accident` entry, marks it as confirmed after a short time, and tells the main system that something serious may have happened.

---

## 6. `violation_detector.py` – Red‑Light Violations

**What this file does**  
`violation_detector.py` checks if a car drove through a red light.

### Important pieces

- **`class Violation`**  
  Represents a single violation.

  It stores:
  - Violation ID
  - Type (here: always red‑light)
  - Which car was involved
  - Which lane it happened in
  - The speed and position when it was detected

  ```python
  from violation_detector import Violation

  v = Violation(violation_id=1, violation_type="RED_LIGHT", lane_name="Lane 1")
  print(v.get_description())
  ```

- **`detect_violations(vehicles)`**  
  Looks at each tracked car and:
  - Checks which lane it is in
  - Checks the current light color for that lane
  - Tests if the car crossed the lane’s stop line while the light was red
  - Uses a minimum speed so slow creeping is not counted as a violation

  ```python
  from violation_detector import ViolationDetector

  detector = ViolationDetector()
  violations = detector.detect_violations(vehicles=current_vehicles)

  for v in violations:
      print("New violation:", v.get_description())
  ```

  *Example:*  
  When the light for Lane 3 is red, and a car quickly crosses over the stop line, this module creates a `Violation` object. The logger then writes a row to `violations.csv`.

> Note: In your current setup, only **red‑light violations** are considered. Other types (like yellow‑light abuse or illegal turns) are not part of the active feature set.

---

## 7. `license_plate_detector.py` – Reading Plate Numbers (Optional)

**What this file does**  
`license_plate_detector.py` tries to read license plates from the video using OCR (Optical Character Recognition).

### Important pieces

- **Background worker**  
  OCR can be slow, so this file runs plate reading in a **separate thread**. The main loop stays smooth, while the worker quietly tries to read plates in the background.

- **Plate detection rules**  
  The code:
  - Takes a small region around the car
  - Looks for shapes that look like a plate (rectangular area of the right size)
  - Cleans the image (grayscale, thresholding)
  - Passes the result to EasyOCR / Tesseract

- **`detect_license_plate(frame, vehicle_bbox)`**  
  Given a car’s box, this function returns:
  - The recognized plate text (if any)
  - A confidence score
  - The exact box where the plate was seen

  ```python
  from license_plate_detector import LicensePlateDetector

  lp_detector = LicensePlateDetector()
  plate_text, conf, plate_bbox = lp_detector.detect_license_plate(frame, vehicle_bbox)

  if plate_text:
      print(f"Detected plate {plate_text} (confidence {conf:.2f})")
  ```

  *Example:*  
  A clear car with a readable plate enters the scene. After a few frames, the OCR worker recognizes "ABC123" with good confidence. The system then displays "ABC123" next to that car and logs it in `license_plates.csv`.

---

## 8. `event_logger.py` – Saving Events and Analytics

**What this file does**  
`event_logger.py` writes important information into CSV files so you can study it later or use it for reports.

### Important pieces

- **Log files it manages**
  - `traffic_events.csv` – all kinds of events (violations, accidents, emergency notifications)
  - `violations.csv` – detailed records for each red‑light violation
  - `accidents.csv` – records for each accident event
  - `license_plates.csv` – records for each recognized license plate

- **`log_violation(violation)`**  
  Writes one line into both the violations log and the general events log.

  ```python
  from event_logger import EventLogger

  logger = EventLogger()

  # "v" is a Violation object from violation_detector
  logger.log_violation(v)
  ```

- **`log_accident(accident, notified=False)`**  
  Writes one line into the accidents log and also into the general events log.

  ```python
  # "acc" is an Accident object from accident_detector
  logger.log_accident(acc, notified=True)
  ```

  *Example:*  
  When a red‑light violation happens in Lane 1 at 08:15, `log_violation` records the time, lane, vehicle ID, and a short description. Later, the dashboard reads this file and shows it in the violations table.

---

## 9. `emergency_notifier.py` – Simulated Emergency Alerts

**What this file does**  
`emergency_notifier.py` simulates calling an emergency hotline when a serious accident is detected.

### Important pieces

- **`notify_accident(accident)`**  
  When called for a confirmed accident, it:
  - Prints a big text block that looks like an emergency SMS
  - Prints another block that looks like a phone call summary (who is being called, what happened, where, and which vehicles are involved)
  - Remembers that this accident has already been notified, so it does not spam the console

  ```python
  from emergency_notifier import EmergencyNotifier

  notifier = EmergencyNotifier()

  # "acc" is an Accident object
  notifier.notify_accident(acc)
  ```

  *Example:*  
  If a collision in Lane 2 is confirmed, the console prints a framed message:
  - Time and hotline number
  - Type: COLLISION
  - Location: South lane
  - IDs of cars involved

- **Cooldown**  
  Uses a timer so repeated updates for the same accident do not trigger new "calls" every frame.

---

## 10. `traffic_controller.py` – Talking to the Arduino

**What this file does**  
`traffic_controller.py` sends simple text commands over USB to the Arduino so the physical lights change with the software.

### Important pieces

- **`connect()`**  
  Opens the serial port defined in `config.ARDUINO_PORT`. If it works, you see a "Connected" message in the console.

- **`send_command(command)`**  
  Sends text like `LANE1` or `AUTO` to the Arduino.

- **`activate_lane(lane_number)` / `activate_lane_by_name(lane_key)`**  
  Helper functions to switch which lane is active.

  ```python
  from traffic_controller import TrafficController

  controller = TrafficController()
  controller.connect()

  # Switch to lane 3
  controller.activate_lane_by_name("LANE3")

  # Later, return to automatic mode
  controller.send_command("AUTO")
  ```

  *Example:*  
  When software decides that it is Lane 3’s turn, it calls `activate_lane_by_name("LANE3")`, which sends `LANE3` to the Arduino. The sketch then turns on Lane 3’s green light and handles the timing.

---

## 11. `config.py` – All Main Settings in One Place

**What this file does**  
`config.py` collects most of the knobs and switches that control USAD’s behavior:
- Which camera to use
- How lanes are drawn
- How long lights stay green/yellow
- How sensitive detection is
- Where logs are saved

### Common settings

- **Camera**  
  `CAMERA_SOURCE`, `CAMERA_WIDTH`, `CAMERA_HEIGHT`, `CAMERA_FPS`

- **Lanes**  
  `LANES` contains the shapes, stop lines, and human‑friendly names for each lane.

- **Timing**  
  `GREEN_TIME`, `YELLOW_TIME`, `MIN_GREEN_TIME`, `MAX_GREEN_TIME`

- **Arduino**  
  `ARDUINO_PORT`, `ARDUINO_BAUDRATE`, `SIMULATE_SIGNALS_WHEN_NO_ARDUINO`

- **Logging**  
  `LOG_DIRECTORY`, names of the CSV files

  ```python
  import config

  print("Camera source:", config.CAMERA_SOURCE)
  print("Log directory:", config.LOG_DIRECTORY)
  ```

  *Example:*  
  If you move the camera or change the intersection, you update the lane polygons and stop lines in `config.py` so red‑light detection still lines up with the actual road.

---

## 12. `ocr_smoke_test.py` – Quick Plate‑Reading Test

**What this file does**  
`ocr_smoke_test.py` is a small script that creates fake plate images (like "ABC123") and checks if the OCR engine can read them.

*Example (how to run it):*  
Run:

```bash
cd USAD-Model
python ocr_smoke_test.py
```

If everything is set up correctly, it prints messages like:

> PASS: 3/4 probes recognized

This tells you the OCR part is working at a basic level.

---

## 13. How Everything Works Together – Simple Scenario

1. You start `app.py`.
2. The app opens the camera and (optionally) the Arduino.
3. Every fraction of a second:
   - A frame is read from the camera.
   - The system finds and follows cars.
   - It checks if any car ran a red light or looks like it is in an accident.
   - It updates the on‑screen overlay.
   - It logs important events.
4. If a serious accident is confirmed, a simulated emergency message is printed.
5. When you are done, you close the window. The camera and Arduino are safely released.

This document should give you a clear, non‑technical picture of what each main file does, with example lines of code showing how the main functions are used and how the pieces fit together.
