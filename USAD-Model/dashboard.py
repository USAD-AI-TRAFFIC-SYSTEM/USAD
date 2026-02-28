"""
USAD Dashboard
--------------
Standalone analytics dashboard — no camera, no OpenCV.
Reads CSV/report files from the same directory (or logs/ subfolder).

Run independently:
    python dashboard.py

Or launched automatically from app.py when the Dashboard tab is clicked.
"""

import os
import csv
import tkinter as tk
import math
from PIL import Image
import customtkinter as ctk
from collections import Counter

# ── Paths ──────────────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def find_file(name):
    for d in [BASE_DIR, os.path.join(BASE_DIR, "logs"), os.path.join(BASE_DIR, "assets")]:
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    return None

def load_csv(name):
    p = find_file(name)
    if not p:
        return []
    with open(p, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))

# ── Colors ─────────────────────────────────────────────────────────────────────
C_BG        = "#ffffff"
C_PRIMARY   = "#282828"
C_SECONDARY = "#5a5a5a"
C_MUTED     = "#969696"
C_LABEL     = "#464646"
C_DIVIDER   = "#c8c8c8"
C_WIN_BG    = "#f0f0f0"

LANE_NAMES = {
    "LANE1": "North", "LANE2": "South",
    "LANE3": "East",  "LANE4": "West",
}
LANE_COLORS = {
    "LANE1": "#1565c0", "LANE2": "#2e7d32",
    "LANE3": "#6a1b9a", "LANE4": "#e65100",
    "UNKNOWN": "#757575", "INTERSECTION": "#37474f",
}

ctk.set_appearance_mode("light")
ctk.set_default_color_theme("blue")


# ── Dashboard App ───────────────────────────────────────────────────────────────
class DashboardApp(ctk.CTk):

    def __init__(self):
        super().__init__()
        self.title("Urban Smart Adaptive Detection (USAD)")
        self.configure(fg_color=C_WIN_BG)
        self.after(0, lambda: self.state("zoomed"))

        # Set window icon
        ico_path = find_file("Logo 1.ico")
        if ico_path:
            self.iconbitmap(ico_path)

        # Load data
        self.violations   = load_csv("violations.csv")
        self.accidents    = load_csv("accidents.csv")
        self.traffic      = load_csv("traffic_events.csv")

        self._build_ui()

    # ── UI ────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.grid_rowconfigure(0, weight=0)
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # ── Top bar ──────────────────────────────────────────────────────
        top = ctk.CTkFrame(self, fg_color=C_BG, corner_radius=0, height=64)
        top.grid(row=0, column=0, sticky="ew")
        top.grid_propagate(False)

        # Logo in top bar
        logo_path = find_file("Logo 1.png")
        if logo_path:
            pil_img = Image.open(logo_path)
            self._logo_img = ctk.CTkImage(light_image=pil_img, dark_image=pil_img, size=(40, 40))
            logo_label = ctk.CTkLabel(top, image=self._logo_img, text="")
            logo_label.pack(side="left", padx=(16, 0), pady=12)

        ctk.CTkLabel(top, text="USAD  ·  Analytics Dashboard",
                     font=("Helvetica", 15, "bold"),
                     text_color=C_PRIMARY).pack(side="left", padx=(10, 20), pady=14)

        # Summary pills in top bar
        self._summary_frame = ctk.CTkFrame(top, fg_color="transparent")
        self._summary_frame.pack(side="right", padx=20)
        self._build_summary_pills()

        ctk.CTkFrame(self, fg_color=C_DIVIDER, height=1).grid(
            row=0, column=0, sticky="ew", pady=(64, 0))
        tab_bar = ctk.CTkFrame(self, fg_color=C_BG, corner_radius=0, height=40)
        tab_bar.grid(row=0, column=0, sticky="ew", pady=(105, 0))
        tab_bar.grid_propagate(False)

        # ── Content area ─────────────────────────────────────────────────
        self._content = ctk.CTkFrame(self, fg_color=C_WIN_BG, corner_radius=0)
        self._content.grid(row=1, column=0, sticky="nsew")

        self._pages = {}
        self._tab_btns = {}
        subtabs = ["Overview", "Violations", "Accidents", "Traffic Events"]

        for name in subtabs:
            pg = ctk.CTkFrame(self._content, fg_color=C_WIN_BG, corner_radius=0)
            self._pages[name] = pg

        for name in subtabs:
            b = ctk.CTkButton(tab_bar, text=name, width=130, height=40,
                               corner_radius=0, border_width=0,
                               font=("Helvetica", 11),
                               fg_color=C_BG, hover_color="#f0f0f0",
                               text_color=C_SECONDARY,
                               command=lambda n=name: self._switch(n))
            b.pack(side="left")
            self._tab_btns[name] = b

        # Bottom border under tabs
        ctk.CTkFrame(self, fg_color=C_DIVIDER, height=1).grid(
            row=0, column=0, sticky="ew", pady=(120, 0))

        # Build pages
        self._build_overview(self._pages["Overview"])
        self._build_table(
            self._pages["Violations"], self.violations,
            ["#", "Time", "Type", "Lane", "Vehicle Type", "Speed (km/h)", "Signal"],
            lambda r: [
                r.get("violation_id",""),
                r.get("time",""),
                r.get("violation_type",""),
                LANE_NAMES.get(r.get("lane",""), r.get("lane","")),
                f'{r.get("vehicle_type","")}',
                f'{float(r.get("speed",0)):.1f}' if r.get("speed") else "—",
                r.get("traffic_signal",""),
            ],
            accent="#b71c1c"
        )
        self._build_table(
            self._pages["Accidents"], self.accidents,
            ["#", "Time", "Type", "Lane", "Vehicle IDs", "Duration (s)", "Notified"],
            lambda r: [
                r.get("accident_id",""),
                r.get("time",""),
                r.get("accident_type",""),
                LANE_NAMES.get(r.get("lane",""), r.get("lane","")),
                r.get("vehicle_ids",""),
                f'{float(r.get("duration",0)):.2f}' if r.get("duration") else "—",
                r.get("emergency_notified",""),
            ],
            accent="#e65100"
        )
        self._build_table(
            self._pages["Traffic Events"], self.traffic,
            ["#", "Timestamp", "Type", "Lane", "Vehicle", "Description"],
            lambda r: [
                r.get("event_id",""),
                r.get("timestamp","")[:19].replace("T"," "),
                r.get("event_type",""),
                LANE_NAMES.get(r.get("lane",""), r.get("lane","")),
                f'{r.get("vehicle_type","")} #{r.get("vehicle_id","")}',
                r.get("description","")[:70],
            ],
            accent="#1565c0"
        )

        self._switch("Overview")

    def _build_summary_pills(self):
        pills = [
            (f"{len(self.violations)}",  "Violations",    "#b71c1c"),
            (f"{len(self.accidents)}",   "Accidents",     "#e65100"),
            (f"{len(self.traffic)}",     "Events",        "#1565c0"),
        ]
        for val, label, color in pills:
            f = ctk.CTkFrame(self._summary_frame, fg_color="#f5f5f5", corner_radius=6)
            f.pack(side="left", padx=4)
            ctk.CTkLabel(f, text=val, font=("Helvetica", 13, "bold"),
                         text_color=color).pack(side="left", padx=(10,4), pady=6)
            ctk.CTkLabel(f, text=label, font=("Helvetica", 10),
                         text_color=C_MUTED).pack(side="left", padx=(0,10))

    def _switch(self, name):
        for n, pg in self._pages.items():
            pg.pack_forget()
        self._pages[name].pack(fill="both", expand=True)
        for n, b in self._tab_btns.items():
            if n == name:
                b.configure(fg_color=C_WIN_BG, text_color=C_PRIMARY,
                            font=("Helvetica", 11, "bold"),
                            border_width=0, border_color=C_PRIMARY)
            else:
                b.configure(fg_color=C_BG, text_color=C_SECONDARY,
                            font=("Helvetica", 11), border_width=0)

    # ── Overview page ─────────────────────────────────────────────────────
    def _build_overview(self, parent):
        scroll = ctk.CTkScrollableFrame(parent, fg_color=C_WIN_BG,
                                         scrollbar_button_color=C_DIVIDER)
        scroll.pack(fill="both", expand=True, padx=0, pady=0)

        def section(text):
            row = ctk.CTkFrame(scroll, fg_color=C_WIN_BG)
            row.pack(fill="x", padx=20, pady=(18, 6))
            ctk.CTkLabel(row, text=text, font=("Helvetica", 12, "bold"),
                         text_color=C_LABEL, anchor="w").pack(side="left")
            ctk.CTkFrame(row, height=1, fg_color=C_DIVIDER).pack(
                side="left", fill="x", expand=True, padx=(10, 0), pady=6)

        def card_row(parent, n):
            fr = ctk.CTkFrame(parent, fg_color=C_WIN_BG)
            fr.pack(fill="x", padx=20, pady=0)
            for i in range(n):
                fr.grid_columnconfigure(i, weight=1)
            return fr

        def stat_card(parent, col, title, value, sub="", accent=C_PRIMARY):
            c = ctk.CTkFrame(parent, fg_color=C_BG, corner_radius=10)
            c.grid(row=0, column=col, padx=(0 if col==0 else 8, 0),
                   pady=4, sticky="ew")
            ctk.CTkLabel(c, text=title, font=("Helvetica", 10),
                         text_color=C_MUTED, anchor="w").pack(
                fill="x", padx=14, pady=(12, 0))
            ctk.CTkLabel(c, text=str(value), font=("Helvetica", 26, "bold"),
                         text_color=accent, anchor="w").pack(fill="x", padx=14)
            ctk.CTkLabel(c, text=sub, font=("Helvetica", 9),
                         text_color=C_MUTED, anchor="w").pack(
                fill="x", padx=14, pady=(0, 12))

        # ── NEW: Hollow pie / donut chart helper ──────────────────────────
        def draw_donut(parent, data_dict, colors, name_map=None):
            """Render a hollow pie (donut) chart using tkinter Canvas."""
            SIZE  = 200
            CX, CY = SIZE // 2, SIZE // 2
            R_OUT  = 80
            R_IN   = 48

            total = sum(data_dict.values()) if data_dict else 0
            sorted_data = sorted(data_dict.items(), key=lambda x: -x[1])

            outer = ctk.CTkFrame(parent, fg_color=C_BG, corner_radius=10)
            outer.pack(fill="x", pady=4)

            canvas = tk.Canvas(outer, width=SIZE, height=SIZE,
                               bg=C_BG, highlightthickness=0)
            canvas.pack(side="left", padx=(16, 8), pady=14)

            if total == 0:
                # Empty state: draw a grey ring
                canvas.create_oval(CX - R_OUT, CY - R_OUT, CX + R_OUT, CY + R_OUT,
                                   fill=C_DIVIDER, outline="")
                canvas.create_oval(CX - R_IN, CY - R_IN, CX + R_IN, CY + R_IN,
                                   fill=C_BG, outline="")
                canvas.create_text(CX, CY, text="No data",
                                   font=("Helvetica", 9), fill=C_MUTED)
            else:
                start_angle = 90.0  # start from 12-o'clock position

                for label, cnt in sorted_data:
                    if cnt == 0:
                        continue
                    sweep = 360.0 * cnt / total
                    color = colors.get(label, C_MUTED)

                    # Draw pie slice
                    canvas.create_arc(
                        CX - R_OUT, CY - R_OUT, CX + R_OUT, CY + R_OUT,
                        start=start_angle, extent=-sweep,
                        fill=color, outline=C_BG, width=2,
                        style="pieslice"
                    )
                    start_angle -= sweep

                # White inner circle → creates the donut hole
                canvas.create_oval(
                    CX - R_IN, CY - R_IN, CX + R_IN, CY + R_IN,
                    fill=C_BG, outline=C_BG
                )

                # Center: total count + label
                canvas.create_text(CX, CY - 9, text=str(total),
                                   font=("Helvetica", 17, "bold"), fill=C_PRIMARY)
                canvas.create_text(CX, CY + 10, text="total",
                                   font=("Helvetica", 8), fill=C_MUTED)

            # ── Legend to the right of the canvas ────────────────────────
            legend = ctk.CTkFrame(outer, fg_color="transparent")
            legend.pack(side="left", fill="y", pady=14, padx=(4, 20))

            for label, cnt in sorted_data:
                if cnt == 0:
                    continue
                display = name_map.get(label, label) if name_map else label
                color   = colors.get(label, C_MUTED)
                pct     = f"{100 * cnt / total:.1f}%" if total else "—"

                lg_row = ctk.CTkFrame(legend, fg_color="transparent")
                lg_row.pack(fill="x", pady=4)

                dot = ctk.CTkFrame(lg_row, width=10, height=10,
                                   corner_radius=5, fg_color=color)
                dot.pack(side="left", padx=(0, 7))
                dot.pack_propagate(False)

                ctk.CTkLabel(lg_row, text=display,
                             font=("Helvetica", 10, "bold"),
                             text_color=C_PRIMARY,
                             anchor="w", width=68).pack(side="left")

                ctk.CTkLabel(lg_row, text=str(cnt),
                             font=("Helvetica", 10, "bold"),
                             text_color=color,
                             anchor="e", width=30).pack(side="left")

                ctk.CTkLabel(lg_row, text=pct,
                             font=("Helvetica", 9),
                             text_color=C_MUTED,
                             anchor="e", width=44).pack(side="left")

        def bar_chart(parent, data_dict, colors, name_map=None, bar_color=None):
            """Render a horizontal bar chart from a dict {label: count}."""
            max_val = max(data_dict.values()) if data_dict else 1
            frame = ctk.CTkFrame(parent, fg_color=C_BG, corner_radius=10)
            frame.pack(fill="x", padx=20, pady=4)
            for label, cnt in sorted(data_dict.items(), key=lambda x: -x[1]):
                if cnt == 0:
                    continue
                row = ctk.CTkFrame(frame, fg_color="transparent")
                row.pack(fill="x", padx=14, pady=4)
                display = name_map.get(label, label) if name_map else label
                color   = bar_color or colors.get(label, C_MUTED)
                ctk.CTkLabel(row, text=display, font=("Helvetica", 10, "bold"),
                             text_color=C_PRIMARY, width=90, anchor="w").pack(side="left")
                bar = ctk.CTkProgressBar(row, height=14, corner_radius=4,
                                          fg_color=C_DIVIDER, progress_color=color)
                bar.set(cnt / max_val)
                bar.pack(side="left", fill="x", expand=True, padx=(8, 8))
                ctk.CTkLabel(row, text=str(cnt), font=("Helvetica", 10, "bold"),
                             text_color=color, width=40, anchor="e").pack(side="left")

        # ── Stat cards ────────────────────────────────────────────────────
        section("Summary")
        r1 = card_row(scroll, 4)
        notified = sum(1 for r in self.accidents
                       if r.get("emergency_notified","").strip().lower() == "yes")
        stat_card(r1, 0, "Total Violations",  len(self.violations),  "Red light",        "#b71c1c")
        stat_card(r1, 1, "Total Accidents",   len(self.accidents),   "All COLLISION",    "#e65100")
        stat_card(r1, 2, "Traffic Events",    len(self.traffic),     "Combined log",     C_PRIMARY)
        stat_card(r1, 3, "Emergency Alerts",  notified,              "Notifications sent","#6a1b9a")

        # ── Accidents & Violations by lane — SIDE-BY-SIDE DONUT CHARTS ───
        section("Accidents & Violations by Lane")
        side_row = ctk.CTkFrame(scroll, fg_color=C_WIN_BG)
        side_row.pack(fill="x", padx=20, pady=4)
        side_row.grid_columnconfigure(0, weight=1)
        side_row.grid_columnconfigure(1, weight=1)

        acc_panel = ctk.CTkFrame(side_row, fg_color=C_WIN_BG)
        acc_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        ctk.CTkLabel(acc_panel, text="Accidents by Lane",
                     font=("Helvetica", 10, "bold"),
                     text_color=C_LABEL, anchor="w").pack(fill="x", pady=(0, 4))
        acc_by_lane = Counter(r.get("lane","UNKNOWN") for r in self.accidents)
        draw_donut(acc_panel, dict(acc_by_lane), LANE_COLORS, LANE_NAMES)

        vio_panel = ctk.CTkFrame(side_row, fg_color=C_WIN_BG)
        vio_panel.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        ctk.CTkLabel(vio_panel, text="Violations by Lane",
                     font=("Helvetica", 10, "bold"),
                     text_color=C_LABEL, anchor="w").pack(fill="x", pady=(0, 4))
        vio_by_lane = Counter(r.get("lane","") for r in self.violations)
        draw_donut(vio_panel, dict(vio_by_lane), LANE_COLORS, LANE_NAMES)

        # ── Peak violation hours ──────────────────────────────────────────
        section("Peak Violation Hours")
        vio_by_hour = Counter(r.get("hour","") for r in self.violations)
        top_hours   = dict(sorted(vio_by_hour.items(), key=lambda x: -x[1])[:8])
        hour_labels = {h: f"{h}:00" for h in top_hours}
        bar_chart(scroll, top_hours, {}, hour_labels, bar_color="#b71c1c")

        # ── Vehicle type breakdown ─────────────────────────────────────────
        section("Vehicle Types in Traffic Events")
        vtype = Counter(r.get("vehicle_type","UNKNOWN") for r in self.traffic)
        vtype_colors = {
            "SMALL":"#1565c0","MEDIUM":"#2e7d32",
            "LARGE":"#e65100","UNKNOWN":"#757575"
        }
        max_v  = max(vtype.values()) if vtype else 1
        vt_row = card_row(scroll, len(vtype))
        for i, (vt, cnt) in enumerate(sorted(vtype.items(), key=lambda x: -x[1])):
            c = ctk.CTkFrame(vt_row, fg_color=C_BG, corner_radius=10)
            c.grid(row=0, column=i, padx=(0 if i==0 else 8,0), pady=4, sticky="ew")
            color = vtype_colors.get(vt, C_MUTED)
            top_r = ctk.CTkFrame(c, fg_color="transparent")
            top_r.pack(fill="x", padx=12, pady=(12,4))
            dot = ctk.CTkFrame(top_r, width=10, height=10,
                               corner_radius=5, fg_color=color)
            dot.pack(side="left", padx=(0,6))
            dot.pack_propagate(False)
            ctk.CTkLabel(top_r, text=vt, font=("Helvetica", 10, "bold"),
                         text_color=C_PRIMARY).pack(side="left")
            ctk.CTkLabel(c, text=str(cnt), font=("Helvetica", 22, "bold"),
                         text_color=color, anchor="w").pack(fill="x", padx=12)
            bar = ctk.CTkProgressBar(c, height=6, corner_radius=3,
                                      fg_color=C_DIVIDER, progress_color=color)
            bar.set(cnt / max_v)
            bar.pack(fill="x", padx=12, pady=(4,12))

        # ── Accident duration stats ────────────────────────────────────────
        section("Accident Duration Distribution")
        durations = []
        for r in self.accidents:
            try:
                durations.append(float(r.get("duration",0)))
            except Exception:
                pass
        if durations:
            buckets = {"0s (instant)": 0, "< 1s": 0, "1–5s": 0, "> 5s": 0}
            for d in durations:
                if d == 0:    buckets["0s (instant)"] += 1
                elif d < 1:   buckets["< 1s"] += 1
                elif d <= 5:  buckets["1–5s"] += 1
                else:         buckets["> 5s"] += 1
            dur_colors = {
                "0s (instant)":"#1565c0","< 1s":"#2e7d32",
                "1–5s":"#fb8c00","> 5s":"#b71c1c"
            }
            bar_chart(scroll, buckets, dur_colors)

    # ── Generic table page ────────────────────────────────────────────────
    def _build_table(self, parent, data, columns, row_fn, accent=C_PRIMARY):
        # Header bar
        header = ctk.CTkFrame(parent, fg_color=C_BG, corner_radius=0, height=52)
        header.pack(fill="x")
        header.pack_propagate(False)

        ctk.CTkLabel(header, text=f"{len(data)} records",
                     font=("Helvetica", 12, "bold"),
                     text_color=accent).pack(side="left", padx=18, pady=14)

        search_var = ctk.StringVar()
        ctk.CTkEntry(header, placeholder_text="🔍  Search...",
                     textvariable=search_var, width=220,
                     font=("Helvetica", 11),
                     border_color=C_DIVIDER).pack(side="right", padx=18, pady=10)

        ctk.CTkFrame(parent, fg_color=C_DIVIDER, height=1).pack(fill="x")

        # Column headers
        col_hdr = ctk.CTkFrame(parent, fg_color="#ebebeb",
                               corner_radius=0, height=30)
        col_hdr.pack(fill="x")
        col_hdr.pack_propagate(False)
        col_widths = self._col_weights(columns)
        for col, w in zip(columns, col_widths):
            ctk.CTkLabel(col_hdr, text=col, font=("Helvetica", 9, "bold"),
                         text_color=C_LABEL, anchor="w", width=w).pack(
                side="left", padx=(10,0))

        ctk.CTkFrame(parent, fg_color=C_DIVIDER, height=1).pack(fill="x")

        # Scrollable rows
        body = ctk.CTkScrollableFrame(parent, fg_color=C_WIN_BG,
                                       scrollbar_button_color=C_DIVIDER)
        body.pack(fill="both", expand=True)

        row_widgets = []

        def render(flt=""):
            for w in row_widgets:
                w.destroy()
            row_widgets.clear()
            fl = flt.lower()
            for idx, record in enumerate(data):
                cells = row_fn(record)
                if fl and fl not in " ".join(str(c) for c in cells).lower():
                    continue
                bg = C_BG if idx % 2 == 0 else "#f8f8f8"
                rf = ctk.CTkFrame(body, fg_color=bg, corner_radius=0, height=28)
                rf.pack(fill="x")
                rf.pack_propagate(False)
                for i, (cell, w) in enumerate(zip(cells, col_widths)):
                    tc = accent if i == 2 else (C_MUTED if i == 0 else C_SECONDARY)
                    ctk.CTkLabel(rf, text=str(cell), font=("Helvetica", 9),
                                 text_color=tc, anchor="w", width=w).pack(
                        side="left", padx=(10,0))
                row_widgets.append(rf)

        render()
        search_var.trace_add("write", lambda *_: render(search_var.get()))

    @staticmethod
    def _col_weights(columns):
        """Approximate fixed widths per column based on typical content."""
        defaults = {"#":40,"Time":80,"Timestamp":140,"Type":160,"Lane":70,
                    "Vehicle":80,"Vehicle Type":90,"Vehicle IDs":100,
                    "Speed (km/h)":90,"Signal":60,"Duration (s)":90,
                    "Notified":70,"Description":340,"Events":60}
        return [defaults.get(c, 120) for c in columns]


# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = DashboardApp()
    app.mainloop()