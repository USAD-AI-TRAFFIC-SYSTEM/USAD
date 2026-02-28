# USAD – Full Project Documentation

**For research and client reference.**  
This document explains the project and every file.

---

## 1. Project overview

USAD stands for Urban Smart Adaptive Dispatcher. It is an AI-based traffic management system.

The system uses a camera to watch the intersection. It detects vehicles in each lane using background subtraction, color segmentation, and tracking. It keeps track of the traffic signal state (GREEN, YELLOW, or RED) for each lane. That state can be synced with a real Arduino board that controls physical traffic lights, or the program can simulate the lights on screen when no Arduino is connected.

The system includes a violation detector called E.Y.E. (Eyeing Your Encounter). E.Y.E. detects when a vehicle runs a red light, abuses a yellow light, or makes an illegal turn. The accident detector finds when two or more vehicles collide (are touching or very close) and can also detect stopped vehicles. When an accident is confirmed, the system triggers emergency notifications: it simulates sending an SMS and placing a call to a hotline (for example 911). License plate text is read using EasyOCR in a separate thread so the main video loop does not slow down. All important events (violations, accidents, plates) are written to CSV files, and the program can generate an analytics report with totals and breakdowns.

Every setting that you might want to change (camera, lanes, timing, detection thresholds, log folder, emergency hotline, and so on) is stored in one file: **USAD-Model/config.py**. The application checks this file from time to time and reloads it while running, so you can adjust behavior without restarting.

---

## 2. Repository structure and folder explanation

The project lives under a root folder called **USAD**. Inside it you will find a **README.md** that describes the project, features, and how to install and run it. The **DOCUMENTATION.md** file (this file) explains every part of the project in sentence form. The **requirements.txt** file lists the Python packages the project needs (OpenCV, NumPy, pyserial, EasyOCR).

The **USAD-Model** folder holds the main Python application. That is where the camera loop, vehicle detection, accident detection, violation detection, license plate reading, traffic control logic, and logging all live. Inside USAD-Model you will see **main.py** (the entry point that runs the camera and ties everything together), **config.py** (all configuration), **vehicle_detector.py** (detects and tracks vehicles), **accident_detector.py** (detects collisions and stopped vehicles), **violation_detector.py** (E.Y.E.: red light, yellow abuse, illegal turn), **license_plate_detector.py** (EasyOCR plate reading in a background thread), **traffic_controller.py** (talks to the Arduino over serial), **event_logger.py** (writes CSV logs and the analytics report), **emergency_notifier.py** (simulates SMS and call for accidents), and **ocr_smoke_test.py** (a small test script for OCR, not used by the main program). The **logs** subfolder inside USAD-Model is where the program writes its CSV files and the analytics report; the exact path is set in config.

The **arduino** folder contains the Arduino sketch. The file **traffic_controller.ino** is the program that runs on the Arduino board. It controls four lanes of traffic lights (red, yellow, green for each lane) and listens for commands from the Python program over the serial port (for example “LANE1”, “LANE2”, “AUTO”).

---

## 3. Modules and libraries used

The project uses OpenCV (imported as **cv2**) for video capture, image processing, background subtraction, and drawing (lane regions, bounding boxes, text on the screen). NumPy is used for arrays and for geometry (for example testing if a point is inside a polygon). The **pyserial** library is used to open the serial port and send commands to the Arduino; the port name and baud rate come from config. **EasyOCR** is used for reading license plate text from vehicle images; it runs in a separate thread so the main loop is not blocked. The rest of the dependencies are from Python’s standard library: **threading** and **queue** for the async OCR worker and the frame grabber, **csv** and **datetime** and **time** for logging and timestamps, **logging** for messages, **os** for file paths, and **re** for cleaning up OCR text. The requirements.txt file typically specifies opencv-python, numpy, pyserial, and easyocr with minimum versions.

---

## 4. USAD-Model – contents of each file in sentences

### 4.1 config.py

This file holds all configuration in one place. It does not contain any detection or control logic; it only defines constants and paths.

At the top it figures out the project root folder and then defines the Arduino settings: which serial port to use (for example COM5), the baud rate, and the timeout. Next it defines the camera: which camera or video source to use, the width and height of the image, and the target frame rate. Then it defines the four lanes (LANE1 through LANE4). For each lane it stores a name (North, South, East, West), a polygon region in pixel coordinates, a stop line (used for violation detection), the direction (vertical or horizontal), and the command string sent to the Arduino (LANE1, LANE2, and so on). It also defines the intersection center as a polygon; that is used to decide if a vehicle or accident is in the middle of the intersection.

The file then defines timing: how many seconds the green light lasts, how many seconds the yellow light lasts, and the minimum and maximum green times used for adaptive timing. It has settings for when the program runs without Arduino (simulation mode). After that come the vehicle detection settings: minimum and maximum blob area to count as a vehicle, background subtractor history and threshold, whether to use color filtering and color segmentation, and the HSV color ranges used to detect car-colored blobs. There are also settings that restrict detection to certain regions (lanes or intersection ROI). The tracking section defines how far a blob can move and still be matched to the same vehicle, smoothing parameters, and how long a track can be lost before it is removed. Another block is dedicated to collision detection: the distance or gap in pixels that counts as “touching” (for both mask-based and bounding-box-based checks), extra pixels required before a collision is considered resolved, confidence frames, and clear delay. Violation detection has its own thresholds: how many pixels past the stop line counts as crossing on red, speed thresholds for red and yellow. Logging is configured with the folder where CSV files and the report are saved and the names of those files. Finally, the emergency notification section sets the hotline number, whether to simulate SMS and call, and the cooldown between notifications for the same accident.

The main idea is that you change behavior by editing this file; the application reloads it periodically so you do not have to restart.

---

### 4.2 main.py

This file is the entry point of the application. It is responsible for opening the camera, running the processing pipeline on each frame, drawing the overlay on the video, handling keyboard input, and shutting down cleanly.

It imports OpenCV, NumPy, time, sys, os, importlib, and threading. It also imports the other modules: traffic_controller, vehicle_detector, accident_detector, violation_detector, license_plate_detector, event_logger, and emergency_notifier.

The file defines a class called **LatestFrameGrabber**. The grabber runs in its own thread. That thread continuously reads frames from the camera and keeps only the latest one in memory. The main loop then takes that latest frame when it is ready, instead of waiting for the camera inside the heavy processing. That way the video does not freeze when detection or OCR is slow.

The main class is **USAD**. When USAD is created, it creates one instance of each component: the traffic controller (for the Arduino), the vehicle detector, the accident detector, the violation detector, the license plate detector, the event logger, and the emergency notifier. It also initializes the current traffic phase (GREEN or YELLOW), the active lane, and the phase start time so the signal cycle can be updated every frame.

The **initialize_camera** method opens the camera using the source and resolution from config (on Windows it often uses DSHOW). **initialize_arduino** tries to connect to the Arduino on the port given in config; if it fails, the program will run in simulation mode and draw the signals on screen only.

The **process_frame** method is the heart of the loop. It takes the current frame and first gets the list of detected and tracked vehicles from the vehicle detector. Then it runs the accident detector on those vehicles and gets the list of confirmed accidents. For each confirmed accident that has not been notified yet, it calls the emergency notifier and logs the accident to the event logger. Next it runs the violation detector on the vehicles (using the current signal state for each lane) and logs each new violation. Optionally it submits license plate crops to the license plate detector (which works in the background). Then it updates the traffic signal cycle (green and yellow timing, and which lane is active) so that the model’s idea of the lights stays in sync with the Arduino or the software simulation. Finally it draws the overlay (lanes, vehicles, accidents, violations, status text) on the frame and returns it for display.

The **update_signal_cycle** method advances the phase (GREEN then YELLOW) and the active lane using the same timing as the Arduino (green and yellow durations from config). It tells the violation detector what the current signal is for each lane so that red-light and yellow-light violations can be judged correctly.

The **activate_lane** method is called when the user presses a number key (1–4) or when the system decides to give green to a specific lane. It sets the active lane, sets the phase to GREEN, updates the violation detector’s signal state, and sends the corresponding command (LANE1, LANE2, and so on) to the Arduino if it is connected.

The **print_statistics** method is called when the user presses S or when the program is shutting down. It asks the event logger for the violation and accident analytics (which are computed from the CSV files). It prints those numbers so they match what is in the analytics report. Then it calls the event logger to generate the full analytics report file.

The **handle_keyboard** method interprets key presses: Q quits, R resets the detectors and notifier, B resets the background learning of the vehicle detector, A switches to auto mode, 1–4 activate a lane, S prints statistics and generates the report, and F toggles fullscreen.

The **run** method opens the camera and optionally starts the frame grabber. Then it runs a loop: get the latest frame, call process_frame, show the result in a window, and handle any key press. When the user quits or closes the window, it runs print_statistics once more and then releases the camera and closes the window.

In one sentence: every frame goes through capture, then vehicle detection, then accident and violation detection and logging, then optional plate submission, then signal update, then drawing and key handling.

---

### 4.3 vehicle_detector.py

This file detects and tracks vehicles in the video and assigns each vehicle to a lane. It also provides speed and direction for the violation detector.

It uses OpenCV, NumPy, collections.deque, time, and config.

The **Vehicle** class represents one tracked vehicle. Each vehicle has an id, a center point, a bounding box, and an area. It stores the last observed center and box so that when detections are missing for a moment (for example when two cars overlap), the track does not drift away. It keeps a short history of positions and timestamps so that speed and direction can be computed. It has a current_lane (which lane or the intersection it is in), a vehicle_type (derived from area, for example SMALL, MEDIUM, LARGE), and optional license_plate text from the OCR. The methods **get_speed** and **get_direction** use the position history and are used by the violation detector.

The **VehicleDetector** class does the actual detection and tracking. It uses OpenCV’s background subtractor (MOG2) to find moving regions. Optionally it applies color masks (HSV ranges from config) so that only car-colored pixels count; that reduces false blobs. The detector can be restricted to an intersection or lane region (ROI) so that only blobs inside that region become vehicles. It finds contours in the foreground mask, filters them by area (between min and max from config), and may split or merge blobs. Each new blob is either matched to an existing vehicle track (by distance and overlap) or starts a new track. When two vehicles are close or overlapping, special logic (crowded and overlap handling) is used to keep bounding boxes stable and avoid swapping IDs. The detector smooths the center and bounding box over time to reduce jitter. For each vehicle it tests whether the center lies inside each lane polygon or the intersection polygon (using OpenCV’s point-in-polygon test) to set current_lane. The method **get_vehicle_count_by_lane** returns how many vehicles are in each lane; that is used for adaptive traffic timing. The **reset** method clears all tracks and can optionally rebuild the background model.

In short: the file turns raw frames into a list of Vehicle objects with lane, bbox, and speed, using config for all thresholds.

---

### 4.4 accident_detector.py

This file detects collisions (two or more vehicles touching or very close) and optionally stopped vehicles. It confirms accidents over several frames, merges duplicate or overlapping collisions, and drives the emergency notifications.

It uses OpenCV, NumPy, time, config, and the Vehicle class from vehicle_detector.

The **Accident** class holds the type (for example COLLISION), the list of involved vehicles, the location (a point), the time it was first detected, whether it has been confirmed (enough consecutive frames with evidence), and whether the emergency notifier has already been called. It figures out which lane the accident is in by testing the location point against the intersection polygon first, then each lane polygon.

The **AccidentDetector** keeps a dictionary of all current accidents. For each frame it runs **\_detect_collisions**. That method looks at every pair of vehicles. It computes the gap between them (either the distance between bounding boxes or, when available, the gap between the actual segmented blobs). The config defines how small the gap must be to count as a collision: COLLISION_OBJECT_GAP_PX for blob gaps and COLLISION_BBOX_TOUCH_PX for box gaps. There is also a release threshold (COLLISION_RELEASE_EXTRA_PX) so that the collision is only cleared when the vehicles have separated by more than that. If the gap is small enough, the pair is recorded as a collision edge. All edges are then grouped (connected components) so that a pile-up of three or more vehicles becomes one accident. New collision evidence is matched to existing accidents (by vehicle IDs and location) so the same crash does not create multiple alerts. Accidents that share vehicles or are close in time and space are merged into one. When the vehicles have been far apart for longer than the configured clear delay, the accident is marked resolved and can be removed. The method **get_confirmed_accidents** returns the list of accidents that have been seen for enough frames; the main program uses that list to trigger notifications and to log. The **reset** method clears all accidents and internal state.

So: the file decides when two or more vehicles are “touching” using pixel thresholds from config, groups them into accidents, confirms and merges them, and exposes the confirmed list for notification and logging.

---

### 4.5 violation_detector.py

This file implements E.Y.E. (Eyeing Your Encounter): it detects red-light violations, yellow-light abuse, and illegal turns using the current signal for each lane and each vehicle’s position and speed.

It uses OpenCV, NumPy, time, config, and the Vehicle class from vehicle_detector.

The **Violation** class stores the type (RED_LIGHT_VIOLATION, YELLOW_ABUSE, ILLEGAL_TURN), the vehicle that committed it, the lane, the traffic signal at the time, the location, the timestamp, and the vehicle’s speed. The method **get_description** returns a short text for logging and display.

The **ViolationDetector** keeps the current signal state for each lane (GREEN, YELLOW, or RED). The main program calls **set_traffic_signal(lane, signal)** whenever the signal changes so that the detector always knows the real state. The method **detect_violations** takes the list of vehicles. For each vehicle it checks the signal of the vehicle’s lane. If the signal is RED and the vehicle has crossed the stop line (by more than the threshold in config), it creates a red-light violation. If the signal is YELLOW and the vehicle is going fast and is far from the stop line (so it could have stopped safely), it can create a yellow-light abuse. Illegal turn uses the vehicle’s direction versus the lane. To avoid reporting the same violation many times, the detector keeps a set of which vehicles have already been checked for which violation type. The method **get_statistics** returns the total count and counts by type from memory; the analytics report and terminal stats, however, use the event logger’s CSV so that they stay correct even after a reset. The **reset** method clears the violation list and the set of checked vehicles.

In one sentence: the file compares each vehicle’s position and speed to the current signal and the stop line and creates one violation per vehicle per type when the rules are broken.

---

### 4.6 license_plate_detector.py

This file reads license plate text from vehicle images using EasyOCR. To avoid slowing the main video loop, all OCR work is done in a separate thread; the main thread only submits jobs and picks up results when they are ready.

It uses OpenCV, NumPy, re, threading, queue, time, and config. It optionally imports EasyOCR; if EasyOCR is not installed, the detector disables itself.

The **LicensePlateDetector** has a queue and a worker thread. The main thread calls **submit_async** with a vehicle id, the full frame, and the vehicle’s bounding box. The detector crops a small image around the vehicle (with a little padding), puts that crop and the vehicle id into the queue, and returns immediately. The worker thread takes jobs from the queue, runs EasyOCR on the crop, cleans the text (strip, normalize spaces, optional regex), and stores the result (plate text, confidence, and box) in a dictionary under the vehicle id. The main thread can later call **get_result(vehicle_id)** to see if a result is ready; if so it gets the text and confidence. Config options control how often to try OCR (every N frames), how many vehicles per frame, and a cooldown per vehicle so the same car is not sent to OCR too often.

So: the file offloads OCR to a background thread via a queue so that the main loop never waits for EasyOCR.

---

### 4.7 traffic_controller.py

This file handles all communication with the Arduino over the serial port. It does not implement detection or signal logic; it only opens the port, sends commands, and reads replies.

It uses the serial module (pyserial), time, logging, and config. The port name, baud rate, and timeout are read from config.

The **TrafficController** class has a **connect** method that opens the serial port and optionally reads the Arduino’s startup message. **disconnect** closes the port. **send_command** takes a string (for example "LANE1" or "AUTO"), sends it with a newline, and enforces a short cooldown so commands are not sent too fast. If the Arduino sends a reply, it is read and logged. **activate_lane** takes a lane number (1–4) and sends the corresponding LANEx command. **activate_lane_by_name** takes a lane key from config (e.g. "LANE1") and sends the command defined in config for that lane. **set_auto_mode** sends "AUTO" so the Arduino goes back to automatic cycling. **get_current_state** returns whether the port is open, whether we are in auto mode, and the current lane. **read_messages** reads any lines waiting in the serial buffer. So the file is a thin wrapper around pyserial; all connection details come from config.

---

### 4.8 event_logger.py

This file writes every violation, accident, and license plate event to CSV files and generates the analytics report. The statistics shown in the terminal when you press S or at shutdown are read from these same CSV files, so the numbers always match the report.

It uses the csv, os, datetime, and time modules, plus config and the Violation and Accident classes.

When the **EventLogger** is created, it creates the log directory if needed and calls **\_initialize_log_files**. That method overwrites the CSV files with a header row only (timestamp, event type, lane, vehicle id, and so on) so each run starts with fresh logs. **log_violation** appends one row to the violations CSV (with timestamp, type, vehicle id, lane, signal, speed, location) and also appends a row to the general traffic_events CSV. **log_accident** appends one row to the accidents CSV (timestamp, type, lane, vehicle ids, duration, whether emergency was notified, location). **log_license_plate** appends to the license_plates CSV. **get_violation_analytics** reads the violations CSV (optionally only rows within a time window), counts by type and by lane, finds high-risk lanes and peak hours, and returns a dictionary. **get_accident_analytics** reads the accidents CSV, counts by type and lane, and counts how many had emergency notified. **generate_report** calls both analytics methods and writes a text file (analytics_report.txt) with sections for violations and accidents; that is the same data that is printed when you press S.

So: the file is the single source of truth for what happened; everything is written to CSV and the report is generated from that.

---

### 4.9 emergency_notifier.py

This file simulates sending an SMS and placing an emergency call when an accident is confirmed. It does not actually send SMS or place calls; it only prints formatted messages to the console.

It uses time, datetime, config, and the Accident class.

The **EmergencyNotifier** keeps a list of all notifications sent and the last notification time per accident (for cooldown). When the main program calls **notify_accident** with a confirmed accident, the notifier checks whether that accident has already been notified and whether the cooldown (from config) has passed. If it is okay to notify, it prints a formatted “SMS” block (time, hotline, accident type, location, vehicle count) and a “call” block with the same information. It sets the accident’s notified flag to True and appends the notification to its history. Config can turn SMS simulation and call simulation on or off and set the hotline number and cooldown. **get_notification_count** returns how many notifications have been sent. **reset** clears the history (for example when the user presses R).

So: the file ensures each accident triggers at most one round of SMS and call simulation, with cooldown, and only prints to the console.

---

### 4.10 ocr_smoke_test.py

This is a small standalone script used to test that EasyOCR and the license plate extraction work. It is not used by the main application loop. You can run it manually to verify that OCR is installed and can read plate-like text from an image or the camera.

---

## 5. Arduino – traffic_controller.ino

This file is the program that runs on the Arduino board. It controls four lanes of traffic lights (each lane has green, yellow, and red) and listens for commands from the Python program over the serial port.

At the top the file defines which Arduino pin is used for each light. Lane 1 (North) uses pins 2 (green), 3 (yellow), and 4 (red). Lane 2 (South) uses 5, 6, and 7. Lane 3 (East) uses 8, 9, and 10. Lane 4 (West) uses 11, 12, and 13. Then it defines the timing: green stays on for 25000 milliseconds (25 seconds) and yellow for 4000 milliseconds (4 seconds). The red light is not timed separately; when a lane is not the active one, its red is on.

The **setup** function initializes the serial port at 9600 baud, sets all those pins as outputs, turns every light off, and then turns on the green for lane 1 and red for the others. It prints a ready message.

The **loop** function runs over and over. Each time it checks if a command has arrived on the serial port. If so, it calls **process_command**, which reads a line, trims and converts to uppercase, and then compares it to "LANE1", "LANE2", "LANE3", "LANE4", or "AUTO". If it is LANE1, it calls **activate_lane(0)**; for LANE2 it uses 1, for LANE3 it uses 2, for LANE4 it uses 3. That switches immediately to that lane with green on and resets the phase timer. If the command is AUTO, the Arduino sets a flag so that it will cycle through lanes automatically. After handling the command, **loop** calls **update_traffic_signal**. That function looks at how much time has passed since the current phase started. If we are in the green phase and the green time has elapsed, it switches to yellow and resets the timer. If we are in the yellow phase and the yellow time has elapsed, it moves to the next lane (in auto mode) or keeps the same lane (when controlled by the model), sets the phase back to green, and resets the timer. **update_display** turns all lights off and then turns on the correct light for the active lane (green or yellow) and turns on red for the other three lanes. **all_lights_off** sets every pin to LOW.

So: the Arduino runs the physical lights with 25 seconds green and 4 seconds yellow per lane, and the Python program sends LANE1–LANE4 or AUTO over serial at 9600 baud. The Python config uses the same 25 and 4 seconds so the model and the hardware stay in sync.

---

## 6. Logs folder

The folder where logs are written is set in config (LOG_DIRECTORY). By default it is **USAD-Model/logs**.

Inside that folder the program writes several CSV files: **violations.csv** (one row per violation with timestamp, type, vehicle, lane, signal, speed, location), **accidents.csv** (one row per accident with timestamp, type, lane, vehicle ids, duration, whether emergency was notified, location), **traffic_events.csv** (a general log of events), and **license_plates.csv** (timestamp, vehicle id, plate text, confidence, location). When you press S or when the program exits, it also writes **analytics_report.txt**, which contains the same totals and breakdowns that were just printed in the terminal (because both the terminal stats and the report are computed from the CSV files).

---

## 7. Data flow in sentences

The main program gets one frame from the camera (or from the frame grabber). It passes that frame to the vehicle detector, which returns a list of Vehicle objects, each with a lane, bounding box, and speed. The main program then passes that list to the accident detector, which returns any new or updated accidents. For each confirmed accident that has not been notified yet, the main program calls the emergency notifier (which prints the SMS and call messages) and logs the accident to the event logger. The event logger appends a row to the accidents CSV and updates the traffic_events CSV. Next the main program passes the vehicle list to the violation detector, which knows the current signal for each lane. The violation detector returns any new violations (red light, yellow abuse, illegal turn). The main program logs each violation to the event logger, which appends to the violations CSV. Optionally the main program sends a crop of each vehicle to the license plate detector; the detector runs OCR in a background thread and later the main program can read the result and log it. The main program then updates the traffic signal cycle (green and yellow timing and which lane is active) so that the on-screen state (and the violation detector’s state) matches the Arduino or the software simulation. It also sends lane commands to the Arduino when the active lane changes. Finally the main program draws the overlay (lanes, boxes, labels, status) on the frame and shows it. When the user presses S or quits, the main program asks the event logger for the violation and accident analytics (by reading the CSV files), prints them, and asks the event logger to write the analytics report file. So the flow is: frame → vehicles → accidents → notifications and accident logging → violations → violation logging → optional plate jobs → signal update → Arduino commands → draw → keys; and on demand, CSV is read and the report is written.

---

## 8. Why object IDs sometimes flicker (e.g. 1 and 2)

The numbers you see on the screen (1, 2, 3, …) are **track IDs**: each tracked vehicle gets one ID when it is first detected, and that ID is meant to stay with the same car over time. When IDs **flicker** (e.g. the same box keeps switching between 1 and 2), it is almost always **ID swap**: two vehicles (or two blobs) and two tracks, and the assignment of “which detection belongs to which track” flips from frame to frame.

**Why it happens**

- The tracker assigns each current detection to an existing track (or creates a new track) using **distance** and **overlap (IOU)** between the track’s box and the detection’s box.
- When **two vehicles are close** or **one vehicle is sometimes split into two blobs**, the distances and IOUs can be very similar for “track 1 ↔ detection A” and “track 1 ↔ detection B”. From one frame to the next, small changes in the segmentation or position can make the assignment prefer “1→B, 2→A” instead of “1→A, 2→B”, so the labels swap and you see 1 and 2 flicker.
- **Pressing R (Reset)** clears all tracks and sets the next ID back to 1. After that, the first two vehicles get IDs 1 and 2 again. If they are close and the assignment is unstable, they can keep swapping 1 and 2.

**What was changed to reduce flicker**

1. **Assignment hysteresis**  
   For each track, the pair (track, detection) that has the **smallest distance** (i.e. the detection closest to where that track was last frame) gets an extra **IOU bonus** in the sort order. So the tracker prefers to keep the same vehicle matched to the same detection instead of swapping when scores are close. The bonus is set in config as **`TRACK_ASSIGNMENT_HYSTERESIS_IOU_BONUS`** (default 0.15).

2. **2×2 assignment when only two tracks and two detections**  
   When there are exactly two tracks and two detections and both tracks could match both detections, the code no longer assigns greedily. It compares the two possible assignments (1→A & 2→B vs 1→B & 2→A) and chooses the one with **lower total distance**. That keeps the same physical car with the same ID more often and reduces flicker between 1 and 2.

**If flicker is still strong**

- Increase **`TRACK_MATCH_MIN_IOU_WHEN_CROWDED`** and **`TRACK_MATCH_MIN_IOU_WHEN_OVERLAP`** in config so that when cars are close, the tracker requires more overlap before assigning; that reduces mistaken swaps.
- Increase **`TRACK_ASSIGNMENT_HYSTERESIS_IOU_BONUS`** (e.g. to 0.2–0.25) so that “keep same assignment” is preferred even more.
- Check that one car is not being split into two blobs (e.g. by reflection or mask); if so, tuning **`MIN_VEHICLE_AREA`** or the background/color thresholds may help.

---

*End of documentation.*
