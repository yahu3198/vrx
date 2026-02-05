#!/usr/bin/env python3
"""
IMU Fault Injection Node for VRX USV Simulation

This node subscribes to a clean IMU topic and republishes with injected faults.
Supports 2 fault types:
    1. Stuck-at-Fault: Frozen sensor readings
    2. Saturation: Output clipped at max/min values

Note: The VRX IMU already has comprehensive noise modeling including:
    - Gaussian noise
    - Bias (static and dynamic)
    - Bias drift
    - Quantization
Therefore, this node focuses on fault modes NOT covered by the existing noise model.

Author: USV Fault Injection Research
License: MIT
"""

import rclpy
from sensor_msgs.msg import Imu

import numpy as np
import copy

from sensor_fault_injection.base_fault_node import BaseFaultInjectionNode
from sensor_fault_injection.fault_types import IMUFaultType, IMU_FAULT_DESCRIPTIONS


class IMUFaultInjectionNode(BaseFaultInjectionNode):
    """
    ROS 2 Node for injecting faults into IMU sensor data.
    
    Subscribes to clean IMU data and republishes with configurable faults.
    """
    
    def __init__(self):
        super().__init__('imu_fault_injection_node')
        
        # ============================================================
        # Setup Publisher and Subscriber
        # ============================================================
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.subscription = self.create_subscription(
            Imu,
            input_topic,
            self._imu_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Imu,
            output_topic,
            10
        )
        
        # Logging
        self.get_logger().info(f'IMU Fault Injection Node initialized')
        self.get_logger().info(f'  Input topic: {input_topic}')
        self.get_logger().info(f'  Output topic: {output_topic}')
        self.get_logger().info(f'  Fault enabled: {self.get_parameter("fault_enabled").value}')
        self.get_logger().info(f'  Fault type: {IMUFaultType(self.get_parameter("fault_type").value).name}')
    
    def _declare_sensor_specific_parameters(self):
        """Declare IMU-specific parameters"""
        
        # Topic configuration
        self.declare_parameter('input_topic', '/wamv/sensors/imu/imu/data')
        self.declare_parameter('output_topic', '/wamv/sensors/imu/imu/data_faulty')
        
        # ----- Saturation Parameters -----
        # Angular velocity limits (rad/s)
        # Typical MEMS gyro range: ±250 to ±2000 deg/s
        # ±250 deg/s ≈ ±4.36 rad/s
        # ±2000 deg/s ≈ ±34.9 rad/s
        self.declare_parameter('saturation.angular_velocity.max_x', 4.36)   # rad/s
        self.declare_parameter('saturation.angular_velocity.max_y', 4.36)   # rad/s
        self.declare_parameter('saturation.angular_velocity.max_z', 4.36)   # rad/s
        
        # Linear acceleration limits (m/s²)
        # Typical MEMS accelerometer range: ±2g to ±16g
        # ±2g ≈ ±19.6 m/s²
        # ±16g ≈ ±156.9 m/s²
        self.declare_parameter('saturation.linear_acceleration.max_x', 19.6)  # m/s²
        self.declare_parameter('saturation.linear_acceleration.max_y', 19.6)  # m/s²
        self.declare_parameter('saturation.linear_acceleration.max_z', 19.6)  # m/s²
        
        # Whether to log saturation events
        self.declare_parameter('saturation.log_events', True)
    
    def _get_fault_type_name(self, fault_type: int) -> str:
        """Get IMU fault type name"""
        try:
            return IMUFaultType(fault_type).name
        except ValueError:
            return f"UNKNOWN({fault_type})"
    
    def _imu_callback(self, msg: Imu):
        """IMU message callback"""
        fault_type = self.get_parameter('fault_type').value
        faulty_msg = self.process_message(msg, fault_type)
        self.publisher.publish(faulty_msg)
    
    def _apply_sensor_specific_fault(self, msg: Imu, fault_type: int) -> Imu:
        """Apply IMU-specific fault"""
        
        if fault_type == IMUFaultType.STUCK:
            return self.apply_stuck_fault(msg)
        elif fault_type == IMUFaultType.SATURATION:
            return self._apply_saturation(msg)
        else:
            return msg
    
    # ================================================================
    # IMU Fault Implementation: Saturation
    # ================================================================
    
    def _apply_saturation(self, msg: Imu) -> Imu:
        """
        Apply saturation fault.
        
        Simulates sensor output clipping when measurements exceed sensor range.
        
        Real-world causes:
        - Extreme motion (collision, high-speed maneuver)
        - Motion exceeding sensor measurement range
        - Sensor configured with wrong range setting
        
        Effects:
        - Angular velocity clipped to ±max_angular_velocity
        - Linear acceleration clipped to ±max_linear_acceleration
        - Covariance increased to indicate degraded measurement
        """
        faulty_msg = copy.deepcopy(msg)
        
        # Get saturation limits
        max_ang_x = self.get_parameter('saturation.angular_velocity.max_x').value
        max_ang_y = self.get_parameter('saturation.angular_velocity.max_y').value
        max_ang_z = self.get_parameter('saturation.angular_velocity.max_z').value
        
        max_acc_x = self.get_parameter('saturation.linear_acceleration.max_x').value
        max_acc_y = self.get_parameter('saturation.linear_acceleration.max_y').value
        max_acc_z = self.get_parameter('saturation.linear_acceleration.max_z').value
        
        log_events = self.get_parameter('saturation.log_events').value
        
        saturated = False
        
        # Apply saturation to angular velocity
        original_ang = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        
        faulty_msg.angular_velocity.x = np.clip(msg.angular_velocity.x, -max_ang_x, max_ang_x)
        faulty_msg.angular_velocity.y = np.clip(msg.angular_velocity.y, -max_ang_y, max_ang_y)
        faulty_msg.angular_velocity.z = np.clip(msg.angular_velocity.z, -max_ang_z, max_ang_z)
        
        clipped_ang = [
            faulty_msg.angular_velocity.x,
            faulty_msg.angular_velocity.y,
            faulty_msg.angular_velocity.z
        ]
        
        if original_ang != clipped_ang:
            saturated = True
            if log_events:
                self.get_logger().warn(
                    f'Angular velocity saturated: '
                    f'[{original_ang[0]:.3f}, {original_ang[1]:.3f}, {original_ang[2]:.3f}] -> '
                    f'[{clipped_ang[0]:.3f}, {clipped_ang[1]:.3f}, {clipped_ang[2]:.3f}] rad/s'
                )
        
        # Apply saturation to linear acceleration
        original_acc = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        
        faulty_msg.linear_acceleration.x = np.clip(msg.linear_acceleration.x, -max_acc_x, max_acc_x)
        faulty_msg.linear_acceleration.y = np.clip(msg.linear_acceleration.y, -max_acc_y, max_acc_y)
        faulty_msg.linear_acceleration.z = np.clip(msg.linear_acceleration.z, -max_acc_z, max_acc_z)
        
        clipped_acc = [
            faulty_msg.linear_acceleration.x,
            faulty_msg.linear_acceleration.y,
            faulty_msg.linear_acceleration.z
        ]
        
        if original_acc != clipped_acc:
            saturated = True
            if log_events:
                self.get_logger().warn(
                    f'Linear acceleration saturated: '
                    f'[{original_acc[0]:.3f}, {original_acc[1]:.3f}, {original_acc[2]:.3f}] -> '
                    f'[{clipped_acc[0]:.3f}, {clipped_acc[1]:.3f}, {clipped_acc[2]:.3f}] m/s²'
                )
        
        # If saturation occurred, increase covariance to indicate degraded measurement
        if saturated:
            # Increase covariance significantly when saturated
            # Original covariance is multiplied by a large factor
            saturation_covariance_factor = 100.0
            
            # Angular velocity covariance (3x3 row-major)
            if msg.angular_velocity_covariance[0] > 0:
                faulty_msg.angular_velocity_covariance = [
                    c * saturation_covariance_factor if i in [0, 4, 8] else c
                    for i, c in enumerate(msg.angular_velocity_covariance)
                ]
            
            # Linear acceleration covariance (3x3 row-major)
            if msg.linear_acceleration_covariance[0] > 0:
                faulty_msg.linear_acceleration_covariance = [
                    c * saturation_covariance_factor if i in [0, 4, 8] else c
                    for i, c in enumerate(msg.linear_acceleration_covariance)
                ]
        
        return faulty_msg


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = IMUFaultInjectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down IMU Fault Injection Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()