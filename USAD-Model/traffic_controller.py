"""Arduino traffic light controller (serial)."""

import serial
import time
import logging
from typing import Optional
import config

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TrafficController:
    """Controls Arduino traffic lights via serial communication"""
    
    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self.current_lane = None
        self.auto_mode = True
        self.last_command_time = 0
        self.command_cooldown = 0.5  # seconds
        
    def connect(self) -> bool:
        """Connect to Arduino on COM6"""
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
        """Disconnect from Arduino"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logger.info("Disconnected from Arduino")
    
    def send_command(self, command: str) -> bool:
        """
        Send command to Arduino
        
        Args:
            command: Command string (LANE1, LANE2, LANE3, LANE4, AUTO)
            
        Returns:
            True if command sent successfully
        """
        current_time = time.time()
        if current_time - self.last_command_time < self.command_cooldown:
            return False
        
        if not self.serial_port or not self.serial_port.is_open:
            logger.error("Arduino not connected")
            return False
        
        try:
            command = command.strip().upper()
            self.serial_port.write(f"{command}\n".encode('utf-8'))
            self.last_command_time = current_time
            
            time.sleep(0.1)
            if self.serial_port.in_waiting:
                response = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                logger.info(f"Arduino response: {response}")
            
            if command in ["LANE1", "LANE2", "LANE3", "LANE4"]:
                self.current_lane = command
                self.auto_mode = False
            elif command == "AUTO":
                self.auto_mode = True
                self.current_lane = None
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            return False
    
    def activate_lane(self, lane_number: int) -> bool:
        """
        Activate specific lane (1-4)
        
        Args:
            lane_number: Lane number (1, 2, 3, or 4)
            
        Returns:
            True if command sent successfully
        """
        if lane_number not in [1, 2, 3, 4]:
            logger.error(f"Invalid lane number: {lane_number}")
            return False
        
        command = f"LANE{lane_number}"
        logger.info(f"Activating {command}")
        return self.send_command(command)
    
    def activate_lane_by_name(self, lane_key: str) -> bool:
        """
        Activate lane by config key (LANE1, LANE2, LANE3, LANE4)
        
        Args:
            lane_key: Lane key from config (e.g., "LANE1")
            
        Returns:
            True if command sent successfully
        """
        if lane_key not in config.LANES:
            logger.error(f"Invalid lane key: {lane_key}")
            return False
        
        command = config.LANES[lane_key]["arduino_cmd"]
        return self.send_command(command)
    
    def set_auto_mode(self) -> bool:
        """Return to automatic cycling mode"""
        logger.info("Setting AUTO mode")
        return self.send_command("AUTO")
    
    def get_current_state(self) -> dict:
        """Get current controller state"""
        return {
            "connected": self.serial_port and self.serial_port.is_open,
            "auto_mode": self.auto_mode,
            "current_lane": self.current_lane,
            "port": config.ARDUINO_PORT
        }
    
    def read_messages(self) -> list:
        """Read any pending messages from Arduino"""
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
        """Context manager entry"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


# Test function
if __name__ == "__main__":
    print("Testing Traffic Controller...")
    
    with TrafficController() as controller:
        if controller.serial_port and controller.serial_port.is_open:
            print("✓ Connected to Arduino")
            
            # Test each lane
            for i in range(1, 5):
                print(f"\nActivating Lane {i}...")
                controller.activate_lane(i)
                time.sleep(8)  # Wait for green + yellow cycle
            
            # Return to auto mode
            print("\nReturning to AUTO mode...")
            controller.set_auto_mode()
            time.sleep(2)
            
            print("\n✓ Test complete")
        else:
            print("✗ Failed to connect to Arduino")
