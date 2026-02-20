"""Emergency notifications (simulated SMS/call)."""

import time
from datetime import datetime
from typing import Dict, List
import config
from accident_detector import Accident


class EmergencyNotifier:
    """Simulates emergency notifications for accidents"""
    
    def __init__(self):
        self.notification_history: List[Dict] = []
        self.last_notification_time: Dict[int, float] = {}  # accident_id -> timestamp
        
    def notify_accident(self, accident: Accident) -> bool:
        """
        Simulate emergency notification for an accident
        
        Args:
            accident: Accident object
            
        Returns:
            True if notification sent (or simulated)
        """
        if accident.notified:
            return False
        
        if accident.id in self.last_notification_time:
            elapsed = time.time() - self.last_notification_time[accident.id]
            if elapsed < config.NOTIFICATION_COOLDOWN:
                return False
        
        location = accident.location
        lane_name = config.LANES[accident.lane]["name"] if accident.lane in config.LANES else accident.lane
        vehicle_ids = [v.id for v in accident.vehicles]
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        message = f"""
╔══════════════════════════════════════════════════════════════╗
║             EMERGENCY ACCIDENT NOTIFICATION                  ║
╠══════════════════════════════════════════════════════════════╣
║ Time: {timestamp}                                            ║
║ Hotline: {config.EMERGENCY_HOTLINE}                          ║
║                                                              ║
║ ACCIDENT DETAILS:                                            ║
║ - Type: {accident.type:<47}                                  ║
║ - Location: {lane_name} Lane                                 ║
║ - Coordinates: ({location[0]}, {location[1]})                ║
║ - Vehicles Involved: {len(vehicle_ids)}                      ║
║ - Vehicle IDs: {', '.join(map(str, vehicle_ids)):<39}        ║
║                                                              ║
║ IMMEDIATE RESPONSE REQUIRED                                  ║
╚══════════════════════════════════════════════════════════════╝
"""
        
        if config.ENABLE_SMS_SIMULATION:
            print("\n" + "="*70)
            print("📱 SMS NOTIFICATION SENT")
            print("="*70)
            print(message)
            
            self._log_notification("SMS", accident, timestamp)
        
        if config.ENABLE_CALL_SIMULATION:
            print("\n" + "="*70)
            print("📞 EMERGENCY CALL PLACED")
            print("="*70)
            print(f"Calling {config.EMERGENCY_HOTLINE}...")
            print(f"Accident Type: {accident.type}")
            print(f"Location: {lane_name} Lane at coordinates ({location[0]}, {location[1]})")
            print(f"Vehicles Involved: {len(vehicle_ids)}")
            print(f"Duration: {accident.get_duration():.1f} seconds")
            print("="*70 + "\n")
            
            self._log_notification("CALL", accident, timestamp)
        
        accident.notified = True
        self.last_notification_time[accident.id] = time.time()
        
        return True
    
    def _log_notification(self, notification_type: str, accident: Accident, timestamp: str):
        """Log notification to history"""
        notification_record = {
            'type': notification_type,
            'timestamp': timestamp,
            'accident_id': accident.id,
            'accident_type': accident.type,
            'lane': accident.lane,
            'location': accident.location,
            'vehicle_count': len(accident.vehicles),
            'hotline': config.EMERGENCY_HOTLINE
        }
        
        self.notification_history.append(notification_record)
    
    def get_notification_count(self) -> int:
        """Get total number of notifications sent"""
        return len(self.notification_history)
    
    def get_notifications_by_type(self, notification_type: str) -> List[Dict]:
        """Get notifications of specific type"""
        return [n for n in self.notification_history if n['type'] == notification_type]
    
    def get_recent_notifications(self, seconds: int = 300) -> List[Dict]:
        """Get notifications from last N seconds"""
        cutoff = time.time() - seconds
        recent = []
        
        for notification in self.notification_history:
            try:
                notif_time = datetime.strptime(notification['timestamp'], '%Y-%m-%d %H:%M:%S').timestamp()
                if notif_time >= cutoff:
                    recent.append(notification)
            except:
                pass
        
        return recent
    
    def print_notification_summary(self):
        """Print summary of all notifications"""
        print("\n" + "="*70)
        print("EMERGENCY NOTIFICATION SUMMARY")
        print("="*70)
        
        if not self.notification_history:
            print("No emergency notifications sent.")
        else:
            total = len(self.notification_history)
            sms_count = len(self.get_notifications_by_type("SMS"))
            call_count = len(self.get_notifications_by_type("CALL"))
            
            print(f"Total Notifications: {total}")
            print(f"  - SMS: {sms_count}")
            print(f"  - Calls: {call_count}")
            print(f"\nHotline: {config.EMERGENCY_HOTLINE}")
            
            # Show recent notifications
            recent = self.get_recent_notifications(600)  # Last 10 minutes
            if recent:
                print(f"\nRecent Notifications (last 10 minutes): {len(recent)}")
                for notif in recent[-5:]:  # Show last 5
                    print(f"  - [{notif['timestamp']}] {notif['type']} for Accident #{notif['accident_id']} in {notif['lane']}")
        
        print("="*70 + "\n")
    
    def reset(self):
        """Reset notification history"""
        self.notification_history.clear()
        self.last_notification_time.clear()


# Test function
if __name__ == "__main__":
    print("Testing Emergency Notifier...")
    
    notifier = EmergencyNotifier()
    
    # Create a mock accident
    class MockVehicle:
        def __init__(self):
            self.id = 123
            self.vehicle_type = "SEDAN"
    
    class MockAccident:
        def __init__(self):
            self.id = 1
            self.type = "COLLISION"
            self.lane = "LANE1"
            self.location = (640, 360)
            self.vehicles = [MockVehicle()]
            self.notified = False
            self.confirmed = True
        
        def get_duration(self):
            return 15.5
    
    mock_accident = MockAccident()
    
    # Test notification
    success = notifier.notify_accident(mock_accident)
    
    if success:
        print("✓ Emergency notification sent successfully")
        notifier.print_notification_summary()
    else:
        print("✗ Failed to send notification")
