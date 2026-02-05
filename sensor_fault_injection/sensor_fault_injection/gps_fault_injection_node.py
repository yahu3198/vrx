#!/usr/bin/env python3
"""
GPS Fault Injection Node for VRX USV Simulation

This node subscribes to a clean GPS topic and republishes with injected faults.
Supports 5 fault types:
    1. Outage/Dropout: Complete signal loss
    2. Jump/Step Error: Instantaneous position offset
    3. Stuck-at-Fault: Frozen sensor readings
    4. Multipath: Oscillating correlated errors
    5. Degraded Rate: Reduced update frequency

Author: USV Fault Injection Research
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

import numpy as np
from typing import Optional
import copy

# Use absolute imports for ROS 2 package
from sensor_fault_injection.fault_types import GPSFaultType, GPS_FAULT_DESCRIPTIONS
from sensor_fault_injection.base_fault_node import BaseFaultInjectionNode


class GPSFaultInjectionNode(BaseFaultInjectionNode):
    """
    ROS 2 Node for injecting faults into GPS sensor data.
    
    Subscribes to clean GPS data and republishes with configurable faults.
    """
    
    def __init__(self):
        # Initialize base class first
        super().__init__('gps_fault_injection_node')
        
        # ============================================================
        # Initialize GPS-specific State Variables
        # ============================================================
        
        # Jump state
        self.jump_active: bool = False
        self.jump_offset_lat: float = 0.0
        self.jump_offset_lon: float = 0.0
        self.jump_offset_alt: float = 0.0
        
        # Multipath state
        self.multipath_phase: float = 0.0
        
        # ============================================================
        # Setup Publisher and Subscriber
        # ============================================================
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.subscription = self.create_subscription(
            NavSatFix,
            input_topic,
            self._gps_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            NavSatFix,
            output_topic,
            10
        )
        
        # Logging
        self.get_logger().info(f'GPS Fault Injection Node initialized')
        self.get_logger().info(f'  Input topic: {input_topic}')
        self.get_logger().info(f'  Output topic: {output_topic}')
        self.get_logger().info(f'  Fault enabled: {self.get_parameter("fault_enabled").value}')
        self.get_logger().info(f'  Fault type: {GPSFaultType(self.get_parameter("fault_type").value).name}')
    
    def _declare_sensor_specific_parameters(self):
        """Declare GPS-specific parameters"""
        
        # Topic configuration
        self.declare_parameter('input_topic', '/wamv/sensors/gps/gps/fix')
        self.declare_parameter('output_topic', '/wamv/sensors/gps/gps/fix_faulty')
        
        # ----- Outage/Dropout Parameters -----
        self.declare_parameter('outage.probability', 0.1)           # Probability of dropout per message
        self.declare_parameter('outage.duration_sec', 2.0)          # Duration of sustained outage
        self.declare_parameter('outage.mode', 'intermittent')       # 'intermittent' or 'sustained'
        
        # ----- Jump/Step Error Parameters -----
        self.declare_parameter('jump.offset_lat', 0.0001)           # Latitude offset in degrees (~11m)
        self.declare_parameter('jump.offset_lon', 0.0001)           # Longitude offset in degrees (~11m)
        self.declare_parameter('jump.offset_alt', 5.0)              # Altitude offset in meters
        self.declare_parameter('jump.probability', 0.05)            # Probability of jump occurring
        self.declare_parameter('jump.persistent', True)             # If True, offset persists after jump
        
        # ----- Multipath Parameters -----
        self.declare_parameter('multipath.amplitude_m', 5.0)        # Error amplitude in meters
        self.declare_parameter('multipath.frequency_hz', 0.2)       # Oscillation frequency
        self.declare_parameter('multipath.phase_offset', 0.0)       # Phase offset in radians
        self.declare_parameter('multipath.noise_stddev', 1.0)       # Additional random noise
    
    def _get_fault_type_name(self, fault_type: int) -> str:
        """Get GPS fault type name"""
        try:
            return GPSFaultType(fault_type).name
        except ValueError:
            return f"UNKNOWN({fault_type})"
    
    def _reset_fault_state(self):
        """Reset all fault state variables including GPS-specific ones"""
        super()._reset_fault_state()
        self.jump_active = False
        self.jump_offset_lat = 0.0
        self.jump_offset_lon = 0.0
        self.jump_offset_alt = 0.0
    
    def _gps_callback(self, msg: NavSatFix):
        """GPS message callback"""
        fault_type = self.get_parameter('fault_type').value
        faulty_msg, should_publish = self.process_message(msg, fault_type)
        
        if should_publish:
            self.publisher.publish(faulty_msg)
    
    def _apply_sensor_specific_fault(self, msg: NavSatFix, fault_type: int) -> tuple:
        """
        Apply GPS-specific fault.
        
        Returns:
            Tuple of (faulty_msg, should_publish)
        """
        if fault_type == GPSFaultType.OUTAGE:
            return self._apply_outage(msg), True
        elif fault_type == GPSFaultType.JUMP:
            return self._apply_jump(msg), True
        elif fault_type == GPSFaultType.STUCK:
            return self.apply_stuck_fault(msg)
        elif fault_type == GPSFaultType.MULTIPATH:
            return self._apply_multipath(msg), True
        elif fault_type == GPSFaultType.DEGRADED_RATE:
            return self.apply_degraded_rate(msg)
        else:
            return msg, True
    
    # ================================================================
    # Fault Implementation: Outage/Dropout
    # ================================================================
    
    def _apply_outage(self, msg: NavSatFix) -> NavSatFix:
        """
        Apply outage/dropout fault.
        
        Simulates complete GPS signal loss by:
        - Setting status to NO_FIX
        - Setting position values to NaN
        - Setting covariance to infinity
        
        Modes:
        - intermittent: Random dropouts based on probability
        - sustained: Continuous outage for specified duration
        """
        mode = self.get_parameter('outage.mode').value
        probability = self.get_parameter('outage.probability').value
        duration = self.get_parameter('outage.duration_sec').value
        
        faulty_msg, _ = self.apply_dropout(
            msg, 
            self._create_outage_message,
            mode=mode,
            probability=probability,
            duration=duration
        )
        
        return faulty_msg
    
    def _create_outage_message(self, msg: NavSatFix) -> NavSatFix:
        """Create a GPS message representing signal outage"""
        faulty_msg = copy.deepcopy(msg)
        faulty_msg.status.status = NavSatStatus.STATUS_NO_FIX
        faulty_msg.status.service = 0
        faulty_msg.latitude = float('nan')
        faulty_msg.longitude = float('nan')
        faulty_msg.altitude = float('nan')
        faulty_msg.position_covariance = [float('inf')] * 9
        faulty_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return faulty_msg
    
    # ================================================================
    # Fault Implementation: Jump/Step Error
    # ================================================================
    
    def _apply_jump(self, msg: NavSatFix) -> NavSatFix:
        """
        Apply jump/step error fault.
        
        Simulates sudden position discontinuity caused by:
        - Satellite constellation changes
        - Receiver reacquisition
        - Kalman filter reset
        
        Can be configured as:
        - One-time jump (persistent=True): Position stays at new offset
        - Random jumps (persistent=False): Each jump is independent
        """
        faulty_msg = copy.deepcopy(msg)
        persistent = self.get_parameter('jump.persistent').value
        
        if persistent:
            # Persistent jump - once triggered, offset remains
            if not self.jump_active:
                probability = self.get_parameter('jump.probability').value
                if np.random.random() < probability:
                    self.jump_active = True
                    self.jump_offset_lat = self.get_parameter('jump.offset_lat').value
                    self.jump_offset_lon = self.get_parameter('jump.offset_lon').value
                    self.jump_offset_alt = self.get_parameter('jump.offset_alt').value
                    
                    # Add some randomness to the jump direction
                    sign_lat = np.random.choice([-1, 1])
                    sign_lon = np.random.choice([-1, 1])
                    self.jump_offset_lat *= sign_lat
                    self.jump_offset_lon *= sign_lon
                    
                    self.get_logger().info(
                        f'Jump triggered: lat={self.jump_offset_lat:.6f}°, '
                        f'lon={self.jump_offset_lon:.6f}°, alt={self.jump_offset_alt:.2f}m'
                    )
            
            if self.jump_active:
                faulty_msg.latitude += self.jump_offset_lat
                faulty_msg.longitude += self.jump_offset_lon
                faulty_msg.altitude += self.jump_offset_alt
        
        else:
            # Non-persistent - random jumps
            probability = self.get_parameter('jump.probability').value
            if np.random.random() < probability:
                offset_lat = self.get_parameter('jump.offset_lat').value
                offset_lon = self.get_parameter('jump.offset_lon').value
                offset_alt = self.get_parameter('jump.offset_alt').value
                
                # Random direction
                sign_lat = np.random.choice([-1, 1])
                sign_lon = np.random.choice([-1, 1])
                
                faulty_msg.latitude += offset_lat * sign_lat
                faulty_msg.longitude += offset_lon * sign_lon
                faulty_msg.altitude += offset_alt
                
                self.get_logger().debug(f'Random jump applied')
        
        return faulty_msg
    
    # ================================================================
    # Fault Implementation: Multipath
    # ================================================================
    
    def _apply_multipath(self, msg: NavSatFix) -> NavSatFix:
        """
        Apply multipath error fault.
        
        Simulates GPS signal reflection causing:
        - Oscillating position errors
        - Quasi-periodic (sinusoidal + noise) pattern
        - Correlated errors over time
        
        Common in maritime environments near:
        - Harbor structures
        - Large vessels
        - Bridges and offshore platforms
        """
        elapsed = self.get_elapsed_time()
        
        amplitude_m = self.get_parameter('multipath.amplitude_m').value
        frequency = self.get_parameter('multipath.frequency_hz').value
        phase_offset = self.get_parameter('multipath.phase_offset').value
        noise_stddev = self.get_parameter('multipath.noise_stddev').value
        
        faulty_msg = copy.deepcopy(msg)
        
        # Sinusoidal error pattern (quasi-periodic)
        # Different frequencies for lat/lon to create more realistic pattern
        error_lat_m = amplitude_m * np.sin(2 * np.pi * frequency * elapsed + phase_offset)
        error_lon_m = amplitude_m * np.cos(2 * np.pi * frequency * 0.7 * elapsed + phase_offset + 0.5)
        
        # Add random noise component
        error_lat_m += np.random.normal(0, noise_stddev)
        error_lon_m += np.random.normal(0, noise_stddev)
        
        # Convert meters to degrees (approximate)
        # 1 degree latitude ≈ 111,320 meters
        # 1 degree longitude ≈ 111,320 * cos(latitude) meters
        error_lat_deg = error_lat_m / 111320.0
        error_lon_deg = error_lon_m / (111320.0 * np.cos(np.radians(msg.latitude)))
        
        faulty_msg.latitude += error_lat_deg
        faulty_msg.longitude += error_lon_deg
        
        # Increase covariance to reflect uncertainty
        base_covariance = amplitude_m ** 2
        faulty_msg.position_covariance = [
            base_covariance, 0.0, 0.0,
            0.0, base_covariance, 0.0,
            0.0, 0.0, base_covariance
        ]
        faulty_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        
        return faulty_msg


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = GPSFaultInjectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down GPS Fault Injection Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()