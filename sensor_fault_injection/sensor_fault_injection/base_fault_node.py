#!/usr/bin/env python3
"""
Base Fault Injection Node

This module provides the base class for all sensor fault injection nodes.
It contains common functionality shared across GPS, IMU, and other sensor fault injectors.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import numpy as np
from typing import Optional, Any
from abc import ABC, abstractmethod
import copy


class BaseFaultInjectionNode(Node, ABC):
    """
    Abstract base class for sensor fault injection nodes.
    
    Provides common functionality:
    - Parameter management
    - Fault enable/disable
    - Stuck-at-fault implementation
    - Dropout/outage implementation
    - Degraded rate implementation
    - Time management
    
    Subclasses must implement:
    - _declare_sensor_specific_parameters()
    - _apply_sensor_specific_fault()
    - _create_subscriber()
    - _create_publisher()
    """
    
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        # ============================================================
        # Common State Variables
        # ============================================================
        self.start_time: Optional[float] = None
        self.message_count: int = 0
        
        # Stuck-at-fault state
        self.stuck_active: bool = False
        self.stuck_message: Optional[Any] = None
        self.stuck_start_time: Optional[float] = None
        
        # Dropout/outage state
        self.in_outage: bool = False
        self.outage_start_time: Optional[float] = None
        
        # Degraded rate state
        self.degraded_rate_active: bool = False
        self.degraded_rate_start_time: Optional[float] = None
        self.message_skip_counter: int = 0
        self.degraded_rate_messages_published: int = 0
        self.degraded_rate_messages_skipped: int = 0
        
        # Last valid message
        self.last_valid_msg: Optional[Any] = None
        
        # ============================================================
        # Declare Common Parameters
        # ============================================================
        self._declare_common_parameters()
        
        # Declare sensor-specific parameters (implemented by subclass)
        self._declare_sensor_specific_parameters()
        
        # Parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self._parameter_callback)
    
    def _declare_common_parameters(self):
        """Declare parameters common to all sensor fault injection nodes"""
        
        # Fault type selection (sensor-specific values)
        self.declare_parameter('fault_type', 0)
        
        # Fault activation
        self.declare_parameter('fault_enabled', False)
        
        # ----- Common Stuck-at-Fault Parameters -----
        self.declare_parameter('stuck.duration_sec', 5.0)
        self.declare_parameter('stuck.trigger_time', 10.0)
        
        # ----- Common Degraded Rate Parameters -----
        # Target rate as fraction of original (0.5 = half rate, 0.25 = quarter rate)
        self.declare_parameter('degraded_rate.rate_factor', 0.5)
        # Or specify target frequency directly (Hz), set to 0 to use rate_factor
        self.declare_parameter('degraded_rate.target_frequency_hz', 0.0)
        # When to start degraded rate (sim time in seconds)
        self.declare_parameter('degraded_rate.trigger_time', 15.0)
        # How long degraded rate lasts (seconds), 0 = permanent until disabled
        self.declare_parameter('degraded_rate.duration_sec', 10.0)
        # Mode: 'periodic' (regular skipping) or 'random' (probabilistic dropping)
        self.declare_parameter('degraded_rate.mode', 'periodic')
    
    @abstractmethod
    def _declare_sensor_specific_parameters(self):
        """Declare parameters specific to the sensor type. Must be implemented by subclass."""
        pass
    
    @abstractmethod
    def _apply_sensor_specific_fault(self, msg: Any, fault_type: int) -> tuple:
        """
        Apply sensor-specific fault. Must be implemented by subclass.
        
        Returns:
            Tuple of (faulty_msg, should_publish)
        """
        pass
    
    @abstractmethod
    def _get_fault_type_name(self, fault_type: int) -> str:
        """Get the name of a fault type. Must be implemented by subclass."""
        pass
    
    def _parameter_callback(self, params) -> SetParametersResult:
        """Handle dynamic parameter updates"""
        for param in params:
            if param.name == 'fault_type':
                fault_name = self._get_fault_type_name(param.value)
                self.get_logger().info(f'Fault type changed to: {fault_name}')
                self._reset_fault_state()
            elif param.name == 'fault_enabled':
                self.get_logger().info(f'Fault enabled: {param.value}')
                if not param.value:
                    self._reset_fault_state()
        
        return SetParametersResult(successful=True)
    
    def _reset_fault_state(self):
        """Reset all fault state variables"""
        self.stuck_active = False
        self.stuck_message = None
        self.stuck_start_time = None
        self.in_outage = False
        self.outage_start_time = None
        self.degraded_rate_active = False
        self.degraded_rate_start_time = None
        self.message_skip_counter = 0
        self.degraded_rate_messages_published = 0
        self.degraded_rate_messages_skipped = 0
        self.get_logger().debug('Fault state reset')
    
    def get_current_time(self) -> float:
        """Get current time in seconds"""
        return self.get_clock().now().nanoseconds / 1e9
    
    def get_elapsed_time(self) -> float:
        """Get elapsed time since first message"""
        if self.start_time is None:
            return 0.0
        return self.get_current_time() - self.start_time
    
    def process_message(self, msg: Any, fault_type_enum: int) -> tuple:
        """
        Common message processing logic.
        
        Args:
            msg: Original sensor message
            fault_type_enum: Integer fault type value
            
        Returns:
            Tuple of (processed_msg, should_publish)
        """
        # Initialize start time on first message
        if self.start_time is None:
            self.start_time = self.get_current_time()
        
        self.message_count += 1
        
        # Store last valid message
        self.last_valid_msg = copy.deepcopy(msg)
        
        # Check if fault injection is enabled
        fault_enabled = self.get_parameter('fault_enabled').value
        
        if not fault_enabled or fault_type_enum == 0:  # 0 = NONE for all sensors
            return msg, True
        
        # Apply the sensor-specific fault
        return self._apply_sensor_specific_fault(msg, fault_type_enum)
    
    # ================================================================
    # Common Fault Implementations
    # ================================================================
    
    def apply_stuck_fault(self, msg: Any, update_header_func=None) -> tuple:
        """
        Apply stuck-at-fault (common to all sensors).
        
        Simulates hardware failure where:
        - Timestamp continues to update
        - Sensor values remain frozen
        
        Args:
            msg: Current sensor message
            update_header_func: Optional function to update header timestamp
            
        Returns:
            Tuple of (faulty_msg, should_publish)
        """
        current_time = self.get_current_time()
        elapsed_since_start = self.get_elapsed_time()
        
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
                return msg, True
            
            # Return stuck message with updated timestamp
            faulty_msg = copy.deepcopy(self.stuck_message)
            if update_header_func:
                faulty_msg = update_header_func(faulty_msg, msg)
            elif hasattr(faulty_msg, 'header'):
                faulty_msg.header.stamp = msg.header.stamp
            return faulty_msg, True
        
        return msg, True
    
    def apply_dropout(self, msg: Any, create_dropout_msg_func, 
                      mode: str = 'intermittent',
                      probability: float = 0.1,
                      duration: float = 2.0) -> tuple:
        """
        Apply dropout/outage fault (common to all sensors).
        
        Args:
            msg: Current sensor message
            create_dropout_msg_func: Function to create a dropout message
            mode: 'intermittent' or 'sustained'
            probability: Probability of dropout
            duration: Duration of sustained outage
            
        Returns:
            Tuple of (faulty_msg, should_publish)
        """
        current_time = self.get_current_time()
        
        if mode == 'intermittent':
            if np.random.random() < probability:
                self.get_logger().debug('Intermittent dropout triggered')
                return create_dropout_msg_func(msg), True
        
        elif mode == 'sustained':
            if not self.in_outage:
                # Check if we should start an outage
                if np.random.random() < probability * 0.1:
                    self.in_outage = True
                    self.outage_start_time = current_time
                    self.get_logger().info(f'Sustained outage started, duration: {duration}s')
            
            if self.in_outage:
                elapsed = current_time - self.outage_start_time
                if elapsed < duration:
                    return create_dropout_msg_func(msg), True
                else:
                    self.in_outage = False
                    self.outage_start_time = None
                    self.get_logger().info('Sustained outage ended')
        
        return msg, True
    
    def apply_degraded_rate(self, msg: Any) -> tuple:
        """
        Apply degraded update rate fault (common to all sensors).
        
        Simulates reduced sensor update frequency caused by:
        - Computational overload
        - Communication bus congestion
        - Power saving mode
        - Firmware issues
        
        Args:
            msg: Current sensor message
            
        Returns:
            Tuple of (message, should_publish)
            - should_publish=False means skip this message
        """
        current_time = self.get_current_time()
        elapsed = self.get_elapsed_time()
        
        trigger_time = self.get_parameter('degraded_rate.trigger_time').value
        duration = self.get_parameter('degraded_rate.duration_sec').value
        rate_factor = self.get_parameter('degraded_rate.rate_factor').value
        mode = self.get_parameter('degraded_rate.mode').value
        
        # Check if we should start degraded rate
        if not self.degraded_rate_active and elapsed >= trigger_time:
            self.degraded_rate_active = True
            self.degraded_rate_start_time = current_time
            self.message_skip_counter = 0
            self.degraded_rate_messages_published = 0
            self.degraded_rate_messages_skipped = 0
            self.get_logger().info(
                f'Degraded rate fault triggered: {rate_factor*100:.0f}% of original rate'
            )
        
        # Check if degraded rate should end (duration > 0 means it has a limit)
        if self.degraded_rate_active and duration > 0:
            degraded_elapsed = current_time - self.degraded_rate_start_time
            if degraded_elapsed >= duration:
                self.degraded_rate_active = False
                total_msgs = self.degraded_rate_messages_published + self.degraded_rate_messages_skipped
                actual_rate = self.degraded_rate_messages_published / total_msgs if total_msgs > 0 else 0
                self.get_logger().info(
                    f'Degraded rate fault ended. '
                    f'Published: {self.degraded_rate_messages_published}, '
                    f'Skipped: {self.degraded_rate_messages_skipped}, '
                    f'Actual rate: {actual_rate*100:.1f}%'
                )
                self.degraded_rate_start_time = None
                return msg, True
        
        # Apply degraded rate if active
        if self.degraded_rate_active:
            should_publish = False
            
            if mode == 'periodic':
                # Periodic skipping: publish every N-th message
                # rate_factor=0.5 means publish every 2nd message (skip_interval=2)
                # rate_factor=0.25 means publish every 4th message (skip_interval=4)
                if rate_factor > 0:
                    skip_interval = int(round(1.0 / rate_factor))
                else:
                    skip_interval = 1000000  # Effectively skip all
                
                self.message_skip_counter += 1
                
                if self.message_skip_counter >= skip_interval:
                    self.message_skip_counter = 0
                    should_publish = True
            
            elif mode == 'random':
                # Random dropping: probabilistic based on rate_factor
                if np.random.random() < rate_factor:
                    should_publish = True
            
            # Update statistics
            if should_publish:
                self.degraded_rate_messages_published += 1
            else:
                self.degraded_rate_messages_skipped += 1
            
            return msg, should_publish
        
        return msg, True