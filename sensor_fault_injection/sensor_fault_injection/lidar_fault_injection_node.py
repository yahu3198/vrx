#!/usr/bin/env python3
"""
LiDAR Fault Injection Node for VRX USV Simulation

This node subscribes to a clean LiDAR point cloud topic and republishes with injected faults.
Supports 3 fault types:
    1. Increased Noise: Noise amplitude multiplied beyond normal
    2. Reduced Range: Max detection distance decreased
    3. Stuck-at-Fault: Frozen point cloud readings

Real-world causes:
    - Increased Noise: Rain, fog, spray, dust, sensor degradation
    - Reduced Range: Fog, rain, dirty lens, low reflectivity targets
    - Stuck-at-Fault: Hardware failure, firmware crash, communication error

Author: USV Fault Injection Research
License: MIT
"""

import rclpy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import copy
import struct
from typing import List, Optional, Tuple

# Use absolute imports for ROS 2 package
from sensor_fault_injection.base_fault_node import BaseFaultInjectionNode
from sensor_fault_injection.fault_types import LiDARFaultType, LIDAR_FAULT_DESCRIPTIONS


class LiDARFaultInjectionNode(BaseFaultInjectionNode):
    """
    ROS 2 Node for injecting faults into LiDAR point cloud data.
    
    Subscribes to clean point cloud data and republishes with configurable faults.
    """
    
    def __init__(self):
        super().__init__('lidar_fault_injection_node')
        
        # Stuck fault: store the frozen point cloud
        self.stuck_point_cloud: Optional[PointCloud2] = None
        
        # ============================================================
        # Setup Publisher and Subscriber
        # ============================================================
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self._lidar_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )
        
        # Logging
        self.get_logger().info(f'LiDAR Fault Injection Node initialized')
        self.get_logger().info(f'  Input topic: {input_topic}')
        self.get_logger().info(f'  Output topic: {output_topic}')
        self.get_logger().info(f'  Fault enabled: {self.get_parameter("fault_enabled").value}')
        self.get_logger().info(f'  Fault type: {LiDARFaultType(self.get_parameter("fault_type").value).name}')
    
    def _declare_sensor_specific_parameters(self):
        """Declare LiDAR-specific parameters"""
        
        # Topic configuration
        self.declare_parameter('input_topic', '/wamv/sensors/lidars/lidar_wamv_sensor/points')
        self.declare_parameter('output_topic', '/wamv/sensors/lidars/lidar_wamv_sensor/points_faulty')
        
        # ----- Increased Noise Parameters -----
        # Simulates degraded point cloud quality
        #
        # noise_multiplier: Factor to multiply existing noise
        #   1.0 = no change, 2.0 = double noise, 5.0 = 5x noise
        #
        # additional_noise_stddev: Extra Gaussian noise added to each point (meters)
        #   Simulates environmental interference (rain, fog, spray)
        #
        self.declare_parameter('noise.noise_multiplier', 2.0)
        self.declare_parameter('noise.additional_noise_stddev', 0.05)  # meters
        self.declare_parameter('noise.affect_intensity', True)  # Also add noise to intensity
        self.declare_parameter('noise.intensity_noise_stddev', 10.0)  # intensity units
        
        # ----- Reduced Range Parameters -----
        # Simulates decreased detection distance
        #
        # max_range: Maximum detection range (meters)
        #   Points beyond this distance are removed
        #
        # range_noise_stddev: Noise added to range threshold
        #   Creates soft boundary instead of hard cutoff
        #
        # min_intensity_threshold: Remove points with intensity below this
        #   Simulates weak returns being lost in fog/rain
        #
        self.declare_parameter('reduced_range.max_range', 50.0)  # meters
        self.declare_parameter('reduced_range.range_noise_stddev', 2.0)  # meters
        self.declare_parameter('reduced_range.min_intensity_threshold', 0.0)
        self.declare_parameter('reduced_range.gradual_falloff', True)  # Gradual vs hard cutoff
        self.declare_parameter('reduced_range.falloff_start', 0.8)  # Start falloff at 80% of max_range
        
        # ----- Stuck-at-Fault Parameters -----
        # Uses common stuck parameters from base class
        # Additional LiDAR-specific stuck parameters:
        self.declare_parameter('stuck.freeze_transform', True)  # Also freeze frame_id
    
    def _get_fault_type_name(self, fault_type: int) -> str:
        """Get LiDAR fault type name"""
        try:
            return LiDARFaultType(fault_type).name
        except ValueError:
            return f"UNKNOWN({fault_type})"
    
    def _lidar_callback(self, msg: PointCloud2):
        """LiDAR point cloud callback"""
        fault_type = self.get_parameter('fault_type').value
        faulty_msg, should_publish = self.process_message(msg, fault_type)
        
        if should_publish:
            self.publisher.publish(faulty_msg)
    
    def _apply_sensor_specific_fault(self, msg: PointCloud2, fault_type: int) -> tuple:
        """
        Apply LiDAR-specific fault.
        
        Returns:
            Tuple of (faulty_msg, should_publish)
        """
        if fault_type == LiDARFaultType.INCREASED_NOISE:
            return self._apply_increased_noise(msg), True
        elif fault_type == LiDARFaultType.REDUCED_RANGE:
            return self._apply_reduced_range(msg), True
        elif fault_type == LiDARFaultType.STUCK:
            return self._apply_stuck(msg)
        else:
            return msg, True
    
    # ================================================================
    # LiDAR Fault Implementation: Increased Noise
    # ================================================================
    
    def _apply_increased_noise(self, msg: PointCloud2) -> PointCloud2:
        """
        Apply increased noise fault to point cloud.
        
        Simulates degraded point cloud quality caused by:
        - Rain droplets scattering laser
        - Fog particles
        - Sea spray
        - Dust
        - Sensor degradation
        
        Effects:
        - Adds Gaussian noise to point positions (x, y, z)
        - Optionally adds noise to intensity values
        """
        noise_multiplier = self.get_parameter('noise.noise_multiplier').value
        additional_stddev = self.get_parameter('noise.additional_noise_stddev').value
        affect_intensity = self.get_parameter('noise.affect_intensity').value
        intensity_stddev = self.get_parameter('noise.intensity_noise_stddev').value
        
        # Read points from point cloud
        points_list = list(pc2.read_points(msg, field_names=None, skip_nans=False))
        
        if not points_list:
            return msg
        
        # Get field names
        field_names = [field.name for field in msg.fields]
        
        # Find indices of x, y, z and intensity fields
        try:
            x_idx = field_names.index('x')
            y_idx = field_names.index('y')
            z_idx = field_names.index('z')
        except ValueError:
            self.get_logger().warn('Point cloud missing x, y, or z fields')
            return msg
        
        intensity_idx = field_names.index('intensity') if 'intensity' in field_names else None
        
        # Apply noise to each point
        noisy_points = []
        for point in points_list:
            point = list(point)
            
            # Add noise to position
            # Combined noise = multiplier effect + additional noise
            total_stddev = additional_stddev * noise_multiplier
            
            point[x_idx] += np.random.normal(0, total_stddev)
            point[y_idx] += np.random.normal(0, total_stddev)
            point[z_idx] += np.random.normal(0, total_stddev)
            
            # Add noise to intensity if enabled
            if affect_intensity and intensity_idx is not None:
                point[intensity_idx] += np.random.normal(0, intensity_stddev * noise_multiplier)
                point[intensity_idx] = max(0, point[intensity_idx])  # Clamp to non-negative
            
            noisy_points.append(tuple(point))
        
        # Create new point cloud message
        faulty_msg = pc2.create_cloud(msg.header, msg.fields, noisy_points)
        
        return faulty_msg
    
    # ================================================================
    # LiDAR Fault Implementation: Reduced Range
    # ================================================================
    
    def _apply_reduced_range(self, msg: PointCloud2) -> PointCloud2:
        """
        Apply reduced range fault to point cloud.
        
        Simulates decreased detection distance caused by:
        - Fog attenuating laser
        - Rain absorption
        - Dirty/obscured lens
        - Low reflectivity targets
        
        Effects:
        - Removes points beyond max_range
        - Optionally applies gradual falloff (probabilistic removal)
        - Optionally filters by intensity threshold
        """
        max_range = self.get_parameter('reduced_range.max_range').value
        range_noise = self.get_parameter('reduced_range.range_noise_stddev').value
        min_intensity = self.get_parameter('reduced_range.min_intensity_threshold').value
        gradual_falloff = self.get_parameter('reduced_range.gradual_falloff').value
        falloff_start = self.get_parameter('reduced_range.falloff_start').value
        
        # Read points from point cloud
        points_list = list(pc2.read_points(msg, field_names=None, skip_nans=False))
        
        if not points_list:
            return msg
        
        # Get field names
        field_names = [field.name for field in msg.fields]
        
        try:
            x_idx = field_names.index('x')
            y_idx = field_names.index('y')
            z_idx = field_names.index('z')
        except ValueError:
            self.get_logger().warn('Point cloud missing x, y, or z fields')
            return msg
        
        intensity_idx = field_names.index('intensity') if 'intensity' in field_names else None
        
        # Filter points
        filtered_points = []
        falloff_range = max_range * falloff_start
        
        for point in points_list:
            x, y, z = point[x_idx], point[y_idx], point[z_idx]
            
            # Calculate range from sensor origin
            range_dist = np.sqrt(x**2 + y**2 + z**2)
            
            # Apply range noise to threshold
            effective_max_range = max_range + np.random.normal(0, range_noise)
            
            # Check intensity threshold
            if intensity_idx is not None and min_intensity > 0:
                if point[intensity_idx] < min_intensity:
                    continue  # Skip low intensity points
            
            # Apply range filtering
            if gradual_falloff:
                # Gradual falloff: probability of keeping point decreases with range
                if range_dist < falloff_range:
                    # Keep all points within falloff start range
                    filtered_points.append(point)
                elif range_dist < effective_max_range:
                    # Probabilistic removal in falloff zone
                    # Linear falloff from 100% at falloff_range to 0% at max_range
                    keep_probability = 1.0 - (range_dist - falloff_range) / (effective_max_range - falloff_range)
                    if np.random.random() < keep_probability:
                        filtered_points.append(point)
                # Points beyond max_range are dropped
            else:
                # Hard cutoff
                if range_dist < effective_max_range:
                    filtered_points.append(point)
        
        # Log statistics occasionally
        if self.message_count % 100 == 0:
            original_count = len(points_list)
            filtered_count = len(filtered_points)
            reduction_pct = (1 - filtered_count / original_count) * 100 if original_count > 0 else 0
            self.get_logger().debug(
                f'Reduced range: {original_count} → {filtered_count} points '
                f'({reduction_pct:.1f}% removed, max_range={max_range}m)'
            )
        
        # Create new point cloud message
        if filtered_points:
            faulty_msg = pc2.create_cloud(msg.header, msg.fields, filtered_points)
        else:
            # Return empty point cloud with same structure
            faulty_msg = pc2.create_cloud(msg.header, msg.fields, [])
        
        return faulty_msg
    
    # ================================================================
    # LiDAR Fault Implementation: Stuck-at-Fault
    # ================================================================
    
    def _apply_stuck(self, msg: PointCloud2) -> tuple:
        """
        Apply stuck-at-fault to point cloud.
        
        Simulates hardware failure where:
        - Timestamp continues to update
        - Point cloud data remains frozen
        
        Real-world causes:
        - Hardware failure
        - Firmware crash
        - Communication buffer not updating
        - Processing pipeline stuck
        """
        current_time = self.get_current_time()
        elapsed_since_start = self.get_elapsed_time()
        
        trigger_time = self.get_parameter('stuck.trigger_time').value
        duration = self.get_parameter('stuck.duration_sec').value
        freeze_transform = self.get_parameter('stuck.freeze_transform').value
        
        # Check if we should start stuck fault
        if not self.stuck_active and elapsed_since_start >= trigger_time:
            self.stuck_active = True
            self.stuck_start_time = current_time
            self.stuck_point_cloud = copy.deepcopy(msg)
            self.get_logger().info(
                f'LiDAR Stuck-at-fault triggered at t={elapsed_since_start:.2f}s '
                f'(frozen {msg.width * msg.height} points)'
            )
        
        # Check if stuck fault should end
        if self.stuck_active:
            stuck_elapsed = current_time - self.stuck_start_time
            if stuck_elapsed >= duration:
                self.stuck_active = False
                self.stuck_point_cloud = None
                self.stuck_start_time = None
                self.get_logger().info(f'LiDAR Stuck-at-fault ended after {duration}s')
                return msg, True
            
            # Return stuck point cloud with updated timestamp
            faulty_msg = copy.deepcopy(self.stuck_point_cloud)
            faulty_msg.header.stamp = msg.header.stamp  # Update timestamp
            
            if not freeze_transform:
                faulty_msg.header.frame_id = msg.header.frame_id
            
            return faulty_msg, True
        
        return msg, True


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = LiDARFaultInjectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LiDAR Fault Injection Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
