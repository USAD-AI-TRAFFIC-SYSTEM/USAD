import os
import time
import cv2
import numpy as np
import customtkinter as ctk
from PIL import Image

# Import USAD engine from main.py (no side-effects since __name__ != "__main__")
from main import USAD
import config

# ── App settings ─────────────────────────────────────────────────────────────
PANEL_W     = 340          # status panel width  (fixed)
PANEL_H     = 525          # status panel height (fixed)
TAB_H       = 36           # tab buttons height above panel
LOGO_SIZE   = 100          # logo box width & height (square)
LOGO_PAD    = 10           # gap from window edge
TICK_MS     = 16           # ~60 fps frame loop interval

ctk.set_appearance_mode("light")
ctk.set_default_color_theme("blue")


# ── Color palette ─────────────────────────────────────────────────────────────
C_BG        = "#ffffff"
C_PRIMARY   = "#282828"
C_SECONDARY = "#5a5a5a"
C_MUTED     = "#969696"
C_LABEL     = "#464646"
C_DIVIDER   = "#c8c8c8"
C_CONNECTED = "#288c3c"
C_DISCONN   = "#b42828"
C_WIN_BG    = "#1a1a1a"

# Lane display names
LANE_NAMES = {
    "LANE1": "North",
    "LANE2": "South",
    "LANE3": "East",
    "LANE4": "West",
}

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def find_file(name):
    for d in [BASE_DIR, os.path.join(BASE_DIR, "logs"), os.path.join(BASE_DIR, "assets")]:
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    return None


class USADApp(ctk.CTk):
    """Main application window."""

    def __init__(self):
        super().__init__()
        self.title("Urban Smart Adaptive Detection (USAD)")
        self.configure(fg_color=C_WIN_BG)
        self.after(0, lambda: self.state("zoomed"))

        # Set window icon
        ico_path = find_file("Logo 1.ico")
        if ico_path:
            self.iconbitmap(ico_path)

        # ── Instantiate USAD engine ──────────────────────────────────────
        self.usad = USAD()
        self.usad.show_cv_panel = False   # use CTk panel instead
        if not self.usad.initialize_camera():
            self._fatal("Camera initialization failed.")
            return
        self.usad.initialize_arduino()

        # Prime the signal simulation
        if not self.usad._is_arduino_connected() and config.LANES:
            self.usad.activate_lane(list(config.LANES.keys())[0])

        # ── Build UI ─────────────────────────────────────────────────────
        self._build_ui()

        # Full screen
        self.after(0, lambda: self.state("zoomed"))   # maximized on Windows/Linux
        self.attributes("-fullscreen", False)          # not borderless — keeps taskbar

        # ── Start frame loop ─────────────────────────────────────────────
        self._running = True
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(TICK_MS, self._tick)

    # ── UI construction ───────────────────────────────────────────────────
    def _build_ui(self):
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # ── Feed fills the entire window ──────────────────────────────────
        feed_frame = ctk.CTkFrame(self, fg_color="#000000", corner_radius=0)
        feed_frame.grid(row=0, column=0, sticky="nsew")
        feed_frame.grid_rowconfigure(0, weight=1)
        feed_frame.grid_columnconfigure(0, weight=1)

        self._feed_label = ctk.CTkLabel(feed_frame, text="")
        self._feed_label.grid(row=0, column=0, sticky="nsew")

        # ── Tab buttons (Camera View / Dashboard) ────────────────────────
        self._tab_frame = ctk.CTkFrame(self, fg_color=C_BG, corner_radius=6,
                                        width=PANEL_W, height=TAB_H, border_width=0)
        self._tab_frame.pack_propagate(False)
        self._tab_frame.grid_propagate(False)

        PAD   = 6                          # outer padding from frame edges
        GAP   = 4                          # gap between the two buttons
        btn_w = (PANEL_W - PAD * 2 - GAP) // 2

        self._btn_camera = ctk.CTkButton(
            self._tab_frame, text="Camera View", width=btn_w, height=TAB_H - PAD * 2,
            corner_radius=4, border_width=0,
            font=("Helvetica", 11, "bold"),
            fg_color=C_PRIMARY, hover_color="#444444", text_color="#ffffff",
            command=self._tab_camera)
        self._btn_camera.place(x=PAD, y=PAD)

        self._btn_dashboard = ctk.CTkButton(
            self._tab_frame, text="Dashboard", width=btn_w, height=TAB_H - PAD * 2,
            corner_radius=4, border_width=0,
            font=("Helvetica", 11),
            fg_color=C_DIVIDER, hover_color="#bbbbbb", text_color=C_SECONDARY,
            command=self._tab_dashboard)
        self._btn_dashboard.place(x=PAD + btn_w + GAP, y=PAD)

        # ── Status panel floats over the feed ────────────────────────────
        self._panel_frame = ctk.CTkFrame(self, fg_color=C_BG, corner_radius=0,
                                          width=PANEL_W, height=PANEL_H, border_width=0)
        self._panel_frame.pack_propagate(False)
        self._panel_frame.grid_propagate(False)
        self._build_status_panel(self._panel_frame)

        # ── Logo box — top-left corner ───────────────────────────────────
        self._logo_frame = ctk.CTkFrame(self, fg_color=C_BG, corner_radius=6,
                                         width=LOGO_SIZE, height=LOGO_SIZE, border_width=0)
        self._logo_frame.pack_propagate(False)
        self._logo_label = ctk.CTkLabel(self._logo_frame, text="")
        self._logo_label.pack(fill="both", expand=True, padx=6, pady=6)
        self._load_logo()

        # Position after window is drawn
        self.after(200, self._place_panel)
        self.after(200, self._place_logo)

    def _place_panel(self):
        """Place tab bar, status panel, and dashboard — vertically centered."""
        try:
            self.update_idletasks()
            sw      = self.winfo_width()
            sh      = self.winfo_height()
            total_h = TAB_H + PANEL_H
            x_right = sw - PANEL_W - 10
            y_top   = (sh - total_h) // 2

            # Tab bar always visible
            self._tab_frame.place(x=x_right, y=y_top)

            self._x_right = x_right
            self._y_panel = y_top + TAB_H

            # Show status panel by default
            self._panel_frame.place(x=x_right, y=self._y_panel)
        except Exception:
            pass

    def _place_logo(self):
        """Place the logo box in the top-left corner."""
        try:
            self._logo_frame.place(x=LOGO_PAD, y=LOGO_PAD)
        except Exception:
            pass

    def _load_logo(self):
        """Load and display the logo image."""
        import os
        logo_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "assets", "Logo 1.png")
        try:
            img = Image.open(logo_path).convert("RGBA")
            # Fit inside the square while preserving aspect ratio
            img.thumbnail((LOGO_SIZE - 12, LOGO_SIZE - 12), Image.LANCZOS)
            ctk_img = ctk.CTkImage(light_image=img, dark_image=img,
                                   size=(img.width, img.height))
            self._logo_label.configure(image=ctk_img, text="")
            self._logo_label._image = ctk_img
        except FileNotFoundError:
            self._logo_label.configure(text="USAD", font=("Helvetica", 13, "bold"),
                                        text_color=C_PRIMARY)
        except Exception as e:
            print(f"[Logo] Could not load: {e}")

    def _build_status_panel(self, parent):
        """Build all widgets inside the right status panel."""
        p = ctk.CTkScrollableFrame(parent, fg_color=C_BG, scrollbar_button_color=C_DIVIDER,
                                   scrollbar_button_hover_color=C_SECONDARY)
        p.pack(fill="both", expand=True, padx=12, pady=12)

        def divider():
            ctk.CTkFrame(p, height=1, fg_color=C_DIVIDER).pack(fill="x", pady=(8, 6))

        # ── Arduino / FPS  — muted, smallest ────────────────────────────
        row0 = ctk.CTkFrame(p, fg_color="transparent")
        row0.pack(fill="x")
        self._lbl_arduino = ctk.CTkLabel(row0, text="Arduino: --",
                                          font=("Helvetica", 11), text_color=C_MUTED, anchor="w")
        self._lbl_arduino.pack(side="left")
        self._lbl_fps = ctk.CTkLabel(row0, text="FPS: --",
                                      font=("Helvetica", 11), text_color=C_MUTED, anchor="e")
        self._lbl_fps.pack(side="right")

        divider()

        # ── Per-lane stoplight rows ───────────────────────────────────────
        DOT = 18
        DIM = "#dddddd"
        self._lane_lights = {}   # lane_key -> (row_frame, r, y, g, lbl_timer)

        for lane_key in config.LANES.keys():
            row = ctk.CTkFrame(p, fg_color="transparent")
            row.pack(fill="x", pady=3)

            # Lane name label
            name_lbl = ctk.CTkLabel(row, text=LANE_NAMES.get(lane_key, lane_key),
                                     font=("Helvetica", 12, "bold"),
                                     text_color=C_PRIMARY, anchor="w", width=90)
            name_lbl.pack(side="left")

            # 3 circles
            r = ctk.CTkFrame(row, width=DOT, height=DOT,
                              corner_radius=DOT//2, fg_color=DIM)
            r.pack(side="left", padx=(4, 2))
            r.pack_propagate(False)

            y = ctk.CTkFrame(row, width=DOT, height=DOT,
                              corner_radius=DOT//2, fg_color=DIM)
            y.pack(side="left", padx=2)
            y.pack_propagate(False)

            g = ctk.CTkFrame(row, width=DOT, height=DOT,
                              corner_radius=DOT//2, fg_color=DIM)
            g.pack(side="left", padx=(2, 8))
            g.pack_propagate(False)

            # Timer label — only visible for the active lane
            timer_lbl = ctk.CTkLabel(row, text="", font=("Helvetica", 11),
                                      text_color=C_MUTED, anchor="w")
            timer_lbl.pack(side="left")

            self._lane_lights[lane_key] = (r, y, g, timer_lbl)

        # ── Total vehicles  — primary ────────────────────────────────────
        self._lbl_vehicles = ctk.CTkLabel(p, text="Total Vehicles: --",
                                           font=("Helvetica", 16, "bold"),  
                                           text_color=C_PRIMARY, anchor="w")
        self._lbl_vehicles.pack(fill="x", pady=(6, 0))

        divider()

        # ── Two-column table header  — label ─────────────────────────────
        hdr = ctk.CTkFrame(p, fg_color="transparent")
        hdr.pack(fill="x")
        hdr.grid_columnconfigure(0, weight=1)
        hdr.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(hdr, text="Vehicle Count", font=("Helvetica", 12, "bold"),
                     text_color=C_LABEL, anchor="w").grid(row=0, column=0, sticky="w")
        ctk.CTkLabel(hdr, text="Lane Status", font=("Helvetica", 12, "bold"),
                     text_color=C_LABEL, anchor="w").grid(row=0, column=1, sticky="w")

        ctk.CTkFrame(p, height=1, fg_color=C_DIVIDER).pack(fill="x", pady=(3, 2))

        # ── Lane rows  — secondary ───────────────────────────────────────
        self._lane_rows = {}
        for lane_key in config.LANES.keys():
            row = ctk.CTkFrame(p, fg_color="transparent")
            row.pack(fill="x", pady=0)
            row.grid_columnconfigure(0, weight=1)
            row.grid_columnconfigure(1, weight=1)
            l_count = ctk.CTkLabel(row, text=f"{LANE_NAMES.get(lane_key, lane_key)}: --",
                                   font=("Helvetica", 11), text_color=C_SECONDARY, anchor="w")
            l_count.grid(row=0, column=0, sticky="w")
            l_state = ctk.CTkLabel(row, text="--",
                                   font=("Helvetica", 11), text_color=C_SECONDARY, anchor="w")
            l_state.grid(row=0, column=1, sticky="w")
            self._lane_rows[lane_key] = (l_count, l_state)

        # ── Control keys hint ─────────────────────────────────────────────
        divider()

        ctk.CTkLabel(p, text="Control Keys",
                     font=("Helvetica", 9, "bold"),
                     text_color=C_MUTED, anchor="w").pack(fill="x", pady=(0, 1))

        keys = [
            ("[Q]", "Quit",             lambda: (self._show_toast("[Key] Q — shutting down.", duration_ms=1500), print("[Key] Q clicked — shutting down."), self._on_close())),
            ("[R]", "Reset",            lambda: (self._show_toast("[Key] R — resetting all detectors."), print("[Key] R clicked — resetting all detectors."), self._cmd_reset())),
            ("[B]", "Reset BG Learning",lambda: (self._show_toast("[Key] B — resetting background learning."), print("[Key] B clicked — resetting background learning."), self._cmd_reset_bg())),
        ]
        for key, desc, cmd in keys:
            ctk.CTkButton(p,
                          text=f"{key}  {desc}",
                          font=("Helvetica", 9),
                          text_color=C_MUTED,
                          fg_color="transparent",
                          hover_color="#eeeeee",
                          corner_radius=4,
                          height=16,
                          border_width=0,
                          anchor="w",
                          command=cmd).pack(fill="x", pady=(0, 0))

    # ── Frame loop ────────────────────────────────────────────────────────
    def _tick(self):
        if not self._running:
            return
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        ret, frame = self.usad.cap.read()
        if not ret:
            print("[App] Failed to read frame")
            self.after(TICK_MS, self._tick)
            return

        # Process frame (detection, signal logic, draw lane overlays)
        processed = self.usad.process_frame(frame)
        self.usad.check_config_reload()

        # FPS counter
        self.usad.frame_count += 1
        elapsed = time.time() - self.usad.start_time
        if elapsed > 1.0:
            self.usad.fps = self.usad.frame_count / elapsed
            self.usad.frame_count = 0
            self.usad.start_time = time.time()

        # Render frame into CTk label
        self._render_feed(processed)

        # Refresh widgets every 5 frames
        self._refresh_counter = getattr(self, '_refresh_counter', 0) + 1
        if self._refresh_counter >= 5:
            self._refresh_panel()
            self._refresh_counter = 0

        self.after(TICK_MS, self._tick)

    def _render_feed(self, frame: np.ndarray):
        """Convert OpenCV frame → CTkImage filling the entire window."""
        try:
            w = self.winfo_width()
            h = self.winfo_height()
        except Exception:
            return
        if w < 2 or h < 2:
            return
        # Resize with OpenCV (much faster than PIL LANCZOS)
        resized = cv2.resize(frame, (w, h), interpolation=cv2.INTER_LINEAR)
        frame_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        ctk_img = ctk.CTkImage(light_image=img, dark_image=img, size=(w, h))
        self._feed_label.configure(image=ctk_img, text="")
        self._feed_label._image = ctk_img   # prevent GC

    def _refresh_panel(self):
        """Push latest USAD state into the CTk panel widgets."""
        u = self.usad

        # Arduino / FPS
        connected = u._is_arduino_connected()
        arduino_text  = "CONNECTED" if connected else "DISCONNECTED"
        arduino_color = C_CONNECTED if connected else C_DISCONN
        self._lbl_arduino.configure(text=f"Arduino: {arduino_text}", text_color=arduino_color)
        self._lbl_fps.configure(text=f"FPS: {u.fps:.1f}")

        # Active lane + signal
        active_lane   = u.current_active_lane or "-"
        active_signal = (u.violation_detector.current_signals.get(active_lane, "-")
                         if u.current_active_lane else "-")
        now = time.time()
        remaining = 0.0
        if u.current_active_lane:
            if u.current_phase == "GREEN":
                remaining = max(0.0, float(u.lane_green_duration) - (now - u.phase_start_time))
            elif u.current_phase == "YELLOW":
                remaining = max(0.0, float(getattr(config, "YELLOW_TIME", 3)) - (now - u.phase_start_time))

        # Per-lane stoplights
        DIM = "#dddddd"
        for lk, (r, y, g, timer_lbl) in self._lane_lights.items():
            sig = u.violation_detector.current_signals.get(lk, "RED")
            r.configure(fg_color="#e53935" if sig == "RED"    else DIM)
            y.configure(fg_color="#fb8c00" if sig == "YELLOW" else DIM)
            g.configure(fg_color="#43a047" if sig == "GREEN"  else DIM)
            # Show timer only on the active lane
            if lk == active_lane and u.current_active_lane:
                timer_lbl.configure(text=f"{remaining:.1f}s")
            else:
                timer_lbl.configure(text="")

        # Vehicles
        lane_counts = u.vehicle_detector.get_vehicle_count_by_lane()
        total = sum(lane_counts.values())
        self._lbl_vehicles.configure(text=f"Total Vehicles: {total}")

        # Lane rows
        for lane_key, (l_count, l_state) in self._lane_rows.items():
            count = int(lane_counts.get(lane_key, 0))
            state = u._classify_lane_congestion(count)
            state_colors = {"CONGESTED": "#b71c1c", "NON-CONGESTED": "#f57f17", "EMPTY": C_MUTED}
            l_count.configure(text=f"{LANE_NAMES.get(lane_key, lane_key)}: {count}")
            l_state.configure(text=state, text_color=state_colors.get(state, C_SECONDARY))

    # ── Button commands ───────────────────────────────────────────────────
    def _tab_camera(self):
        """Select Camera View tab (already the active view)."""
        self._btn_camera.configure(fg_color=C_PRIMARY, text_color="#ffffff",
                                    font=("Helvetica", 11, "bold"))
        self._btn_dashboard.configure(fg_color=C_DIVIDER, text_color=C_SECONDARY,
                                       font=("Helvetica", 11))

    def _tab_dashboard(self):
        """Launch dashboard.py as a separate window."""
        import subprocess, sys, os
        self._btn_dashboard.configure(fg_color=C_PRIMARY, text_color="#ffffff",
                                       font=("Helvetica", 11, "bold"))
        self._btn_camera.configure(fg_color=C_DIVIDER, text_color=C_SECONDARY,
                                    font=("Helvetica", 11))
        # Reset button appearance back after 300ms
        self.after(300, self._tab_camera)
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard.py")
        subprocess.Popen([sys.executable, script])

    # ── Toast notification ───────────────────────────────────────────────
    def _show_toast(self, message: str, duration_ms: int = 2800):
        """Show a self-dismissing toast card at the bottom-center of the window."""
        # Dismiss any existing toast
        if hasattr(self, "_toast_frame") and self._toast_frame.winfo_exists():
            try:
                self._toast_frame.place_forget()
                self._toast_frame.destroy()
            except Exception:
                pass

        toast = ctk.CTkFrame(self, fg_color=C_PRIMARY, corner_radius=10,
                             border_width=0)
        ctk.CTkLabel(toast, text=message,
                     font=("Helvetica", 11),
                     text_color="#ffffff",
                     wraplength=460,
                     justify="center").pack(padx=18, pady=10)
        self._toast_frame = toast

        def _place():
            try:
                self.update_idletasks()
                tw = toast.winfo_reqwidth()
                th = toast.winfo_reqheight()
                sw = self.winfo_width()
                sh = self.winfo_height()
                x  = (sw - tw) // 2
                y  = sh - th - 24
                toast.place(x=x, y=y)
                toast.lift()
            except Exception:
                pass

        self.after(10, _place)
        self.after(duration_ms, lambda: self._dismiss_toast(toast))

    def _dismiss_toast(self, toast):
        try:
            if toast.winfo_exists():
                toast.place_forget()
                toast.destroy()
        except Exception:
            pass

    def _cmd_auto(self):
        msg = "[App] Auto mode enabled — system will manage signals automatically."
        print(msg)
        self._show_toast(msg)
        if self.usad._is_arduino_connected():
            self.usad.traffic_controller.set_auto_mode()
        else:
            self.usad.software_auto_mode = True

    def _cmd_reset(self):
        msg = "[App] Reset triggered — clearing vehicle, accident, violation, and notifier state."
        print(msg)
        self._show_toast(msg)
        self.usad.vehicle_detector.reset()
        self.usad.accident_detector.reset()
        self.usad.violation_detector.reset()
        self.usad.emergency_notifier.reset()
        self._show_toast("[App] Reset complete.")
        print("[App] Reset complete.")

    def _cmd_reset_bg(self):
        msg = "[App] Background learning reset — detector will re-learn the scene background."
        print(msg)
        self._show_toast(msg)
        try:
            self.usad.vehicle_detector.reset_background()
            print("[App] Background learning reset complete.")
            self._show_toast("[App] Background learning reset complete.")
        except AttributeError:
            print("[App] Background learning reset complete (no explicit reset method found).")
            self._show_toast("[App] Background learning reset complete.")

    def _cmd_stats(self):
        msg = "[App] Printing statistics..."
        print(msg)
        self._show_toast(msg)
        self.usad.print_statistics()

    def _cmd_activate_lane(self, lane_key: str):
        msg = f"[App] Manual override — activating {lane_key} ({LANE_NAMES.get(lane_key, lane_key)})."
        print(msg)
        self._show_toast(msg)
        self.usad.software_auto_mode = False
        self.usad.activate_lane(lane_key)

    # ── Keyboard passthrough ──────────────────────────────────────────────
    def _bind_keys(self):
        self.bind("<KeyPress>", self._on_key)

    def _on_key(self, event):
        key = event.keysym.lower()
        if key in ("q", "escape"):
            msg = "[Key] Q / Escape — shutting down."
            print(msg)
            self._show_toast(msg, duration_ms=1500)
            self._on_close()
        elif key == "r":
            self._show_toast("[Key] R — resetting all detectors.")
            self._cmd_reset()
        elif key == "b":
            self._show_toast("[Key] B — resetting background learning.")
            self._cmd_reset_bg()
        elif key == "a":
            self._show_toast("[Key] A — enabling auto mode.")
            self._cmd_auto()
        elif key == "s":
            self._show_toast("[Key] S — printing statistics.")
            self._cmd_stats()
        elif key == "f":
            self._show_toast("[Key] F — toggling fullscreen.")
            self.usad.toggle_fullscreen()
        elif key in ("1", "2", "3", "4"):
            lane = f"LANE{key}"
            if lane in config.LANES:
                msg = f"[Key] {key} — activating {lane} ({LANE_NAMES.get(lane, lane)})."
                print(msg)
                self._show_toast(msg)
                self._cmd_activate_lane(lane)

    # ── Close ─────────────────────────────────────────────────────────────
    def _on_close(self):
        self._running = False
        try:
            self.usad.print_statistics()
        except Exception:
            pass
        try:
            if self.usad.cap:
                self.usad.cap.release()
        except Exception:
            pass
        try:
            if self.usad._is_arduino_connected():
                self.usad.traffic_controller.disconnect()
        except Exception:
            pass
        try:
            self.destroy()
        except Exception:
            pass

    def _fatal(self, msg: str):
        print(f"[FATAL] {msg}")
        self.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = USADApp()
    app._bind_keys()
    app.mainloop()