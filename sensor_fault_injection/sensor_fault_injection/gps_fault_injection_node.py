#!/usr/bin/env python3
"""
GPS Fault Injection Node for VRX USV Simulation

This node subscribes to a clean GPS topic and republishes with injected faults.
Supports 4 fault types:
    1. Outage/Dropout: Complete signal loss
    2. Jump/Step Error: Instantaneous position offset
    3. Stuck-at-Fault: Frozen sensor readings
    4. Multipath: Oscillating correlated errors

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
from enum import IntEnum
from typing import Optional
import copy


class GPSFaultType(IntEnum):
    """Enumeration of GPS fault types"""
    NONE = 0
    OUTAGE = 1          # Complete signal loss / dropout
    JUMP = 2            # Instantaneous position offset
    STUCK = 3           # Frozen sensor readings
    MULTIPATH = 4       # Oscillating correlated errors


class GPSFaultInjectionNode(Node):
    """
    ROS 2 Node for injecting faults into GPS sensor data.
    
    Subscribes to clean GPS data and republishes with configurable faults.
    """
    
    def __init__(self):
        super().__init__('gps_fault_injection_node')
        
        # ============================================================
        # Declare Parameters
        # ============================================================
        
        # Topic configuration
        self.declare_parameter('input_topic', '/wamv/sensors/gps/gps/fix')
        self.declare_parameter('output_topic', '/wamv/sensors/gps/gps/fix_faulty')
        
        # Fault type selection (0=None, 1=Outage, 2=Jump, 3=Stuck, 4=Multipath)
        self.declare_parameter('fault_type', 0)
        
        # Fault activation
        self.declare_parameter('fault_enabled', False)
        
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
        
        # ----- Stuck-at-Fault Parameters -----
        self.declare_parameter('stuck.duration_sec', 5.0)           # How long sensor stays stuck
        self.declare_parameter('stuck.trigger_time', 10.0)          # When to trigger stuck fault (sim time)
        
        # ----- Multipath Parameters -----
        self.declare_parameter('multipath.amplitude_m', 5.0)        # Error amplitude in meters
        self.declare_parameter('multipath.frequency_hz', 0.2)       # Oscillation frequency
        self.declare_parameter('multipath.phase_offset', 0.0)       # Phase offset in radians
        self.declare_parameter('multipath.noise_stddev', 1.0)       # Additional random noise
        
        # ============================================================
        # Initialize State Variables
        # ============================================================
        
        # General state
        self.start_time: Optional[float] = None
        self.message_count: int = 0
        
        # Outage state
        self.in_outage: bool = False
        self.outage_start_time: Optional[float] = None
        
        # Jump state
        self.jump_active: bool = False
        self.jump_offset_lat: float = 0.0
        self.jump_offset_lon: float = 0.0
        self.jump_offset_alt: float = 0.0
        
        # Stuck state
        self.stuck_active: bool = False
        self.stuck_message: Optional[NavSatFix] = None
        self.stuck_start_time: Optional[float] = None
        
        # Multipath state
        self.multipath_phase: float = 0.0
        
        # Store last valid message
        self.last_valid_msg: Optional[NavSatFix] = None
        
        # ============================================================
        # Setup Publisher and Subscriber
        # ============================================================
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.subscription = self.create_subscription(
            NavSatFix,
            input_topic,
            self.gps_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            NavSatFix,
            output_topic,
            10
        )
        
        # Parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Logging
        self.get_logger().info(f'GPS Fault Injection Node initialized')
        self.get_logger().info(f'  Input topic: {input_topic}')
        self.get_logger().info(f'  Output topic: {output_topic}')
        self.get_logger().info(f'  Fault enabled: {self.get_parameter("fault_enabled").value}')
        self.get_logger().info(f'  Fault type: {GPSFaultType(self.get_parameter("fault_type").value).name}')
    
    def parameter_callback(self, params) -> SetParametersResult:
        """Handle dynamic parameter updates"""
        for param in params:
            if param.name == 'fault_type':
                self.get_logger().info(f'Fault type changed to: {GPSFaultType(param.value).name}')
                # Reset state when fault type changes
                self.reset_fault_state()
            elif param.name == 'fault_enabled':
                self.get_logger().info(f'Fault enabled: {param.value}')
                if not param.value:
                    self.reset_fault_state()
        
        return SetParametersResult(successful=True)
    
    def reset_fault_state(self):
        """Reset all fault state variables"""
        self.in_outage = False
        self.outage_start_time = None
        self.jump_active = False
        self.jump_offset_lat = 0.0
        self.jump_offset_lon = 0.0
        self.jump_offset_alt = 0.0
        self.stuck_active = False
        self.stuck_message = None
        self.stuck_start_time = None
        self.get_logger().debug('Fault state reset')
    
    def get_current_time(self) -> float:
        """Get current time in seconds"""
        return self.get_clock().now().nanoseconds / 1e9
    
    def gps_callback(self, msg: NavSatFix):
        """
        Main callback for processing GPS messages.
        Applies the selected fault type and publishes the result.
        """
        # Initialize start time on first message
        if self.start_time is None:
            self.start_time = self.get_current_time()
        
        self.message_count += 1
        
        # Store last valid message
        self.last_valid_msg = copy.deepcopy(msg)
        
        # Check if fault injection is enabled
        fault_enabled = self.get_parameter('fault_enabled').value
        fault_type = GPSFaultType(self.get_parameter('fault_type').value)
        
        if not fault_enabled or fault_type == GPSFaultType.NONE:
            # Pass through without modification
            self.publisher.publish(msg)
            return
        
        # Apply the selected fault
        faulty_msg = self.apply_fault(msg, fault_type)
        
        # Publish faulty message
        self.publisher.publish(faulty_msg)
    
    def apply_fault(self, msg: NavSatFix, fault_type: GPSFaultType) -> NavSatFix:
        """
        Apply the specified fault type to the GPS message.
        
        Args:
            msg: Original GPS message
            fault_type: Type of fault to apply
            
        Returns:
            Modified GPS message with fault injected
        """
        if fault_type == GPSFaultType.OUTAGE:
            return self.apply_outage(msg)
        elif fault_type == GPSFaultType.JUMP:
            return self.apply_jump(msg)
        elif fault_type == GPSFaultType.STUCK:
            return self.apply_stuck(msg)
        elif fault_type == GPSFaultType.MULTIPATH:
            return self.apply_multipath(msg)
        else:
            return msg
    
    # ================================================================
    # Fault Implementation: Outage/Dropout
    # ================================================================
    
    def apply_outage(self, msg: NavSatFix) -> NavSatFix:
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
        current_time = self.get_current_time()
        mode = self.get_parameter('outage.mode').value
        
        faulty_msg = copy.deepcopy(msg)
        
        if mode == 'intermittent':
            # Random dropout based on probability
            probability = self.get_parameter('outage.probability').value
            if np.random.random() < probability:
                faulty_msg = self.create_outage_message(faulty_msg)
                self.get_logger().debug('Intermittent outage triggered')
        
        elif mode == 'sustained':
            duration = self.get_parameter('outage.duration_sec').value
            
            if not self.in_outage:
                # Check if we should start an outage
                probability = self.get_parameter('outage.probability').value
                if np.random.random() < probability * 0.1:  # Lower probability for starting sustained outage
                    self.in_outage = True
                    self.outage_start_time = current_time
                    self.get_logger().info(f'Sustained outage started, duration: {duration}s')
            
            if self.in_outage:
                elapsed = current_time - self.outage_start_time
                if elapsed < duration:
                    faulty_msg = self.create_outage_message(faulty_msg)
                else:
                    self.in_outage = False
                    self.outage_start_time = None
                    self.get_logger().info('Sustained outage ended')
        
        return faulty_msg
    
    def create_outage_message(self, msg: NavSatFix) -> NavSatFix:
        """Create a GPS message representing signal outage"""
        msg.status.status = NavSatStatus.STATUS_NO_FIX
        msg.status.service = 0
        msg.latitude = float('nan')
        msg.longitude = float('nan')
        msg.altitude = float('nan')
        msg.position_covariance = [float('inf')] * 9
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return msg
    
    # ================================================================
    # Fault Implementation: Jump/Step Error
    # ================================================================
    
    def apply_jump(self, msg: NavSatFix) -> NavSatFix:
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
    # Fault Implementation: Stuck-at-Fault
    # ================================================================
    
    def apply_stuck(self, msg: NavSatFix) -> NavSatFix:
        """
        Apply stuck-at-fault.
        
        Simulates hardware failure where:
        - Timestamp continues to update
        - Position values remain frozen
        
        Triggered after specified time and lasts for specified duration.
        """
        current_time = self.get_current_time()
        elapsed_since_start = current_time - self.start_time
        
        trigger_time = self.get_parameter('stuck.trigger_time').value
        duration = self.get_parameter('stuck.duration_sec').value
        
        # Check if we should start stuck fault
        if not self.stuck_active and elapsed_since_start >= trigger_time:
            self.stuck_active = True
            self.stuck_message = copy.deepcopy(msg)
            self.stuck_start_time = current_time
            self.get_logger().info(f'Stuck-at-fault triggered at t={elapsed_since_start:.2f}s')
        
        # Check if stuck fault should end
        if self.stuck_active:
            stuck_elapsed = current_time - self.stuck_start_time
            if stuck_elapsed >= duration:
                self.stuck_active = False
                self.stuck_message = None
                self.stuck_start_time = None
                self.get_logger().info(f'Stuck-at-fault ended after {duration}s')
                return msg
            
            # Return stuck message with updated timestamp
            faulty_msg = copy.deepcopy(self.stuck_message)
            faulty_msg.header.stamp = msg.header.stamp  # Update timestamp
            return faulty_msg
        
        return msg
    
    # ================================================================
    # Fault Implementation: Multipath
    # ================================================================
    
    def apply_multipath(self, msg: NavSatFix) -> NavSatFix:
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
        current_time = self.get_current_time()
        elapsed = current_time - self.start_time
        
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