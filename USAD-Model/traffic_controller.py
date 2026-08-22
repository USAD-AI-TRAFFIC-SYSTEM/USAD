"""Arduino traffic light controller (serial) — Slave mode.

The Arduino is a dumb slave: it only changes lights when Python sends an
explicit command. Python is the single source of truth for signal state.

Commands sent to Arduino:
  LANE1_GREEN, LANE1_YELLOW
  LANE2_GREEN, LANE2_YELLOW
  LANE3_GREEN, LANE3_YELLOW
  LANE4_GREEN, LANE4_YELLOW
  ALL_RED
  PING
"""

import serial
import time
import logging
from typing import Optional
import config

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TrafficController:
    """Controls Arduino traffic lights via serial communication (slave mode)."""

    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self.current_lane: Optional[str] = None
        self.current_phase: Optional[str] = None  # "GREEN", "YELLOW", "RED"
        self.last_command_time = 0
        self.command_cooldown = 0.05  # 50ms — much faster than before (was 500ms)

    def connect(self) -> bool:
        """Connect to Arduino."""
        try:
            self.serial_port = serial.Serial(
                port=config.ARDUINO_PORT,
                baudrate=config.ARDUINO_BAUDRATE,
                timeout=config.ARDUINO_TIMEOUT
            )
            time.sleep(2)  # Wait for Arduino to initialize
            logger.info(f"Connected to Arduino on {config.ARDUINO_PORT}")

            if self.serial_port.in_waiting:
                startup_msg = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                logger.info(f"Arduino: {startup_msg}")

            return True

        except serial.SerialException as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            return False

    def disconnect(self):
        """Disconnect from Arduino."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logger.info("Disconnected from Arduino")

    def _send(self, command: str) -> bool:
        """Low-level send with minimal cooldown."""
        now = time.time()
        if now - self.last_command_time < self.command_cooldown:
            # If cooldown blocks, wait it out for critical commands
            remaining = self.command_cooldown - (now - self.last_command_time)
            time.sleep(remaining)

        if not self.serial_port or not self.serial_port.is_open:
            return False

        try:
            # Flush any stale input to avoid buffer corruption
            if self.serial_port.in_waiting:
                self.serial_port.reset_input_buffer()

            self.serial_port.write(f"{command}\n".encode('utf-8'))
            self.serial_port.flush()  # Ensure bytes are sent immediately
            self.last_command_time = time.time()

            # Wait for Arduino acknowledgment
            time.sleep(0.03)
            if self.serial_port.in_waiting:
                response = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                logger.debug(f"Arduino: {response}")

            return True

        except Exception as e:
            logger.error(f"Failed to send command '{command}': {e}")
            return False

    # ── Public API: explicit phase control ─────────────────────────────────

    def set_signal(self, lane_key: str, phase: str) -> bool:
        """Set a specific lane to a specific phase (GREEN or YELLOW).
        All other lanes are set to RED by the Arduino automatically.

        Args:
            lane_key: "LANE1", "LANE2", "LANE3", or "LANE4"
            phase: "GREEN" or "YELLOW"

        Returns:
            True if command sent successfully.
        """
        phase = phase.upper()
        lane_key = lane_key.upper()

        if lane_key not in ("LANE1", "LANE2", "LANE3", "LANE4"):
            logger.error(f"Invalid lane key: {lane_key}")
            return False

        if phase not in ("GREEN", "YELLOW"):
            logger.error(f"Invalid phase: {phase}")
            return False

        # Don't re-send if already in this state
        if self.current_lane == lane_key and self.current_phase == phase:
            return True

        command = f"{lane_key}_{phase}"
        success = self._send(command)
        if success:
            self.current_lane = lane_key
            self.current_phase = phase
        return success

    def set_all_red(self) -> bool:
        """Set all lanes to red."""
        success = self._send("ALL_RED")
        if success:
            self.current_lane = None
            self.current_phase = "RED"
        return success

    def ping(self) -> bool:
        """Check if Arduino is responsive."""
        return self._send("PING")

    # ── Legacy API (backward compatible) ──────────────────────────────────

    def send_command(self, command: str) -> bool:
        """Legacy: send raw command string."""
        command = command.strip().upper()

        # Map legacy commands to new API
        if command in ("LANE1", "LANE2", "LANE3", "LANE4"):
            return self.set_signal(command, "GREEN")
        elif command == "AUTO":
            # In slave mode, AUTO is a no-op — Python controls everything
            return self._send("AUTO")
        else:
            return self._send(command)

    def activate_lane(self, lane_number: int) -> bool:
        """Legacy: activate lane by number (1-4), sets to GREEN."""
        if lane_number not in (1, 2, 3, 4):
            logger.error(f"Invalid lane number: {lane_number}")
            return False
        return self.set_signal(f"LANE{lane_number}", "GREEN")

    def activate_lane_by_name(self, lane_key: str) -> bool:
        """Legacy: activate lane by config key, sets to GREEN."""
        if lane_key not in config.LANES:
            logger.error(f"Invalid lane key: {lane_key}")
            return False
        return self.set_signal(lane_key, "GREEN")

    def set_auto_mode(self) -> bool:
        """Legacy: send AUTO (no-op in slave mode, acknowledged by Arduino)."""
        logger.info("Setting AUTO mode (slave: no-op)")
        return self._send("AUTO")

    # ── Utilities ─────────────────────────────────────────────────────────

    def get_current_state(self) -> dict:
        """Get current controller state."""
        return {
            "connected": bool(self.serial_port and self.serial_port.is_open),
            "current_lane": self.current_lane,
            "current_phase": self.current_phase,
            "port": config.ARDUINO_PORT
        }

    def read_messages(self) -> list:
        """Read any pending messages from Arduino."""
        messages = []
        if not self.serial_port or not self.serial_port.is_open:
            return messages
        try:
            while self.serial_port.in_waiting:
                msg = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if msg:
                    messages.append(msg)
                    logger.debug(f"Arduino: {msg}")
        except Exception as e:
            logger.error(f"Error reading messages: {e}")
        return messages

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


# Test function
if __name__ == "__main__":
    print("Testing Traffic Controller (Slave Mode)...")

    with TrafficController() as controller:
        if controller.serial_port and controller.serial_port.is_open:
            print("Connected to Arduino")

            # Test each lane green + yellow
            for i in range(1, 5):
                lane = f"LANE{i}"
                print(f"\n{lane} GREEN...")
                controller.set_signal(lane, "GREEN")
                time.sleep(3)

                print(f"{lane} YELLOW...")
                controller.set_signal(lane, "YELLOW")
                time.sleep(2)

            print("\nALL RED...")
            controller.set_all_red()
            time.sleep(2)

            print("\nTest complete")
        else:
            print("Failed to connect to Arduino")
