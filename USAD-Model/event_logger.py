"""CSV event logging + simple analytics/reporting."""

import csv
import os
from datetime import datetime
from typing import List, Dict, Optional
import time
import config
from violation_detector import Violation
from accident_detector import Accident


class EventLogger:
    """Logs traffic events and generates analytics"""
    
    def __init__(self):
        os.makedirs(config.LOG_DIRECTORY, exist_ok=True)
        
        self.event_log_path = os.path.join(config.LOG_DIRECTORY, config.EVENT_LOG_FILE)
        self.violation_log_path = os.path.join(config.LOG_DIRECTORY, config.VIOLATION_LOG_FILE)
        self.accident_log_path = os.path.join(config.LOG_DIRECTORY, config.ACCIDENT_LOG_FILE)
        
        self._initialize_log_files()
        
        self.last_analytics_update = time.time()
        
    def _initialize_log_files(self):
        """Create log files with headers if they don't exist"""
        if not os.path.exists(self.event_log_path):
            with open(self.event_log_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'event_type', 'event_id', 'lane', 'vehicle_id',
                    'vehicle_type', 'license_plate', 'description', 'location'
                ])
        
        if not os.path.exists(self.violation_log_path):
            with open(self.violation_log_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'date', 'time', 'hour', 'violation_type', 'violation_id',
                    'vehicle_id', 'vehicle_type', 'license_plate', 'lane', 'traffic_signal',
                    'speed', 'location_x', 'location_y'
                ])
        
        if not os.path.exists(self.accident_log_path):
            with open(self.accident_log_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'date', 'time', 'hour', 'accident_type', 'accident_id',
                    'lane', 'vehicle_ids', 'vehicle_count', 'duration', 'emergency_notified',
                    'location_x', 'location_y'
                ])
    
    def log_violation(self, violation: Violation):
        """Log a traffic violation"""
        dt = datetime.fromtimestamp(violation.timestamp)
        
        row = [
            dt.isoformat(),
            dt.strftime('%Y-%m-%d'),
            dt.strftime('%H:%M:%S'),
            dt.hour,
            violation.type,
            violation.id,
            violation.vehicle.id,
            violation.vehicle.vehicle_type,
            violation.license_plate or 'N/A',
            violation.lane,
            violation.traffic_signal,
            round(violation.speed, 2),
            violation.location[0],
            violation.location[1]
        ]
        
        with open(self.violation_log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
        
        self._log_event(
            violation.type,
            violation.id,
            violation.lane,
            violation.vehicle.id,
            violation.vehicle.vehicle_type,
            violation.license_plate,
            violation.get_description(),
            violation.location,
            violation.timestamp
        )
    
    def log_accident(self, accident: Accident, notified: bool = False):
        """Log an accident"""
        dt = datetime.fromtimestamp(accident.detected_time)
        
        vehicle_ids = ','.join([str(v.id) for v in accident.vehicles])
        
        row = [
            dt.isoformat(),
            dt.strftime('%Y-%m-%d'),
            dt.strftime('%H:%M:%S'),
            dt.hour,
            accident.type,
            accident.id,
            accident.lane or 'UNKNOWN',
            vehicle_ids,
            len(accident.vehicles),
            round(accident.get_duration(), 2),
            'Yes' if notified else 'No',
            accident.location[0],
            accident.location[1]
        ]
        
        with open(self.accident_log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
        
        vehicle_id = accident.vehicles[0].id if accident.vehicles else 'N/A'
        vehicle_type = accident.vehicles[0].vehicle_type if accident.vehicles else 'N/A'
        license_plate = accident.vehicles[0].license_plate if accident.vehicles else None
        
        self._log_event(
            'ACCIDENT',
            accident.id,
            accident.lane or 'UNKNOWN',
            vehicle_id,
            vehicle_type,
            license_plate,
            accident.get_description(),
            accident.location,
            accident.detected_time
        )
    
    def _log_event(self, event_type: str, event_id: int, lane: str, vehicle_id: int,
                   vehicle_type: str, license_plate: Optional[str], description: str,
                   location: tuple, timestamp: float):
        """Log a general event"""
        dt = datetime.fromtimestamp(timestamp)
        
        row = [
            dt.isoformat(),
            event_type,
            event_id,
            lane,
            vehicle_id,
            vehicle_type,
            license_plate or 'N/A',
            description,
            f"({location[0]}, {location[1]})"
        ]
        
        with open(self.event_log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
    
    def get_violation_analytics(self, time_window: Optional[int] = None) -> Dict:
        """
        Get violation analytics
        
        Args:
            time_window: Time window in seconds (None for all time)
            
        Returns:
            Dictionary with analytics data
        """
        if not os.path.exists(self.violation_log_path):
            return {}
        
        violations = []
        cutoff_time = time.time() - time_window if time_window else 0
        
        with open(self.violation_log_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                timestamp = datetime.fromisoformat(row['timestamp']).timestamp()
                if timestamp >= cutoff_time:
                    violations.append(row)
        
        if not violations:
            return {'total': 0}
        
        # Count by type
        by_type = {}
        for v in violations:
            vtype = v['violation_type']
            by_type[vtype] = by_type.get(vtype, 0) + 1
        
        # Count by lane
        by_lane = {}
        for v in violations:
            lane = v['lane']
            by_lane[lane] = by_lane.get(lane, 0) + 1
        
        # Count by hour
        by_hour = {}
        for v in violations:
            hour = int(v['hour'])
            by_hour[hour] = by_hour.get(hour, 0) + 1
        
        # Find high-risk intersections (lanes with 3x more violations)
        avg_violations = sum(by_lane.values()) / len(by_lane) if by_lane else 0
        high_risk_lanes = []
        
        for lane, count in by_lane.items():
            if count >= avg_violations * config.HIGH_RISK_VIOLATION_MULTIPLIER:
                high_risk_lanes.append({
                    'lane': lane,
                    'violations': count,
                    'multiplier': round(count / avg_violations, 2) if avg_violations > 0 else 0
                })
        
        # Find peak violation hours
        peak_hours = sorted(by_hour.items(), key=lambda x: x[1], reverse=True)[:3]
        
        return {
            'total': len(violations),
            'by_type': by_type,
            'by_lane': by_lane,
            'by_hour': by_hour,
            'high_risk_lanes': high_risk_lanes,
            'peak_hours': peak_hours,
            'time_window': time_window
        }
    
    def get_accident_analytics(self, time_window: Optional[int] = None) -> Dict:
        """Get accident analytics"""
        if not os.path.exists(self.accident_log_path):
            return {}
        
        accidents = []
        cutoff_time = time.time() - time_window if time_window else 0
        
        with open(self.accident_log_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                timestamp = datetime.fromisoformat(row['timestamp']).timestamp()
                if timestamp >= cutoff_time:
                    accidents.append(row)
        
        if not accidents:
            return {'total': 0}
        
        # Count by type
        by_type = {}
        for a in accidents:
            atype = a['accident_type']
            by_type[atype] = by_type.get(atype, 0) + 1
        
        # Count by lane
        by_lane = {}
        for a in accidents:
            lane = a['lane']
            by_lane[lane] = by_lane.get(lane, 0) + 1
        
        # Emergency notifications
        notified_count = sum(1 for a in accidents if a['emergency_notified'] == 'Yes')
        
        return {
            'total': len(accidents),
            'by_type': by_type,
            'by_lane': by_lane,
            'emergency_notified': notified_count,
            'time_window': time_window
        }
    
    def generate_report(self, output_file: str = 'analytics_report.txt'):
        """Generate a comprehensive analytics report"""
        report_path = os.path.join(config.LOG_DIRECTORY, output_file)
        
        # Get analytics
        violation_analytics = self.get_violation_analytics()
        accident_analytics = self.get_accident_analytics()
        
        with open(report_path, 'w') as f:
            f.write("=" * 70 + "\n")
            f.write("USAD TRAFFIC ANALYTICS REPORT\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 70 + "\n\n")
            
            # Violations
            f.write("VIOLATION STATISTICS\n")
            f.write("-" * 70 + "\n")
            f.write(f"Total Violations: {violation_analytics.get('total', 0)}\n\n")
            
            if violation_analytics.get('by_type'):
                f.write("By Type:\n")
                for vtype, count in violation_analytics['by_type'].items():
                    f.write(f"  - {vtype}: {count}\n")
                f.write("\n")
            
            if violation_analytics.get('by_lane'):
                f.write("By Lane:\n")
                for lane, count in violation_analytics['by_lane'].items():
                    f.write(f"  - {lane}: {count}\n")
                f.write("\n")
            
            if violation_analytics.get('high_risk_lanes'):
                f.write("High-Risk Lanes (3x+ violations):\n")
                for risk in violation_analytics['high_risk_lanes']:
                    f.write(f"  - {risk['lane']}: {risk['violations']} violations ({risk['multiplier']}x average)\n")
                f.write("\n")
            
            if violation_analytics.get('peak_hours'):
                f.write("Peak Violation Hours:\n")
                for hour, count in violation_analytics['peak_hours']:
                    f.write(f"  - {hour}:00: {count} violations\n")
                f.write("\n")
            
            # Accidents
            f.write("\nACCIDENT STATISTICS\n")
            f.write("-" * 70 + "\n")
            f.write(f"Total Accidents: {accident_analytics.get('total', 0)}\n")
            f.write(f"Emergency Notifications: {accident_analytics.get('emergency_notified', 0)}\n\n")
            
            if accident_analytics.get('by_type'):
                f.write("By Type:\n")
                for atype, count in accident_analytics['by_type'].items():
                    f.write(f"  - {atype}: {count}\n")
                f.write("\n")
            
            if accident_analytics.get('by_lane'):
                f.write("By Lane:\n")
                for lane, count in accident_analytics['by_lane'].items():
                    f.write(f"  - {lane}: {count}\n")
            
            f.write("\n" + "=" * 70 + "\n")
        
        print(f"Analytics report saved to: {report_path}")
        return report_path


# Test function
if __name__ == "__main__":
    print("Testing Event Logger...")
    
    logger = EventLogger()
    print(f"✓ Log files initialized in: {config.LOG_DIRECTORY}")
    
    # Test analytics
    v_analytics = logger.get_violation_analytics()
    a_analytics = logger.get_accident_analytics()
    
    print(f"✓ Total violations logged: {v_analytics.get('total', 0)}")
    print(f"✓ Total accidents logged: {a_analytics.get('total', 0)}")
    
    # Generate report
    logger.generate_report()
