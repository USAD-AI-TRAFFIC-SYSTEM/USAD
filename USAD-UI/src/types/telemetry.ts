export interface TelemetryData {
  fps: number;
  camera_source: number;
  arduino_connected: boolean;
  software_auto_mode: boolean;
  current_active_lane: string | null;
  current_phase: "GREEN" | "YELLOW" | "RED";
  phase_time_remaining: number;
  total_vehicles: number;
  lane_counts: Record<string, number>;
  lane_states: Record<string, "CONGESTED" | "NON-CONGESTED" | "EMPTY">;
  lane_signals: Record<string, "GREEN" | "YELLOW" | "RED">;
  adaptive_green_durations: Record<string, number>;
  active_accidents_count: number;
  active_stopped_cars_count: number;
  total_violations_session: number;
  emergency_notifications_count: number;
  detected_license_plates: { text: string; confidence: number }[];
}

export interface LogRecord {
  [key: string]: string;
}

export interface LogSummary {
  violations: {
    total: number;
    by_type?: Record<string, number>;
  };
  accidents: {
    total: number;
    emergency_notified?: number;
    by_type?: Record<string, number>;
  };
  emergency_notifications: number;
}

export const LANE_NAMES: Record<string, string> = {
  LANE1: "North",
  LANE2: "South",
  LANE3: "East",
  LANE4: "West",
};

export const LANE_KEYS = ["LANE1", "LANE2", "LANE3", "LANE4"] as const;
