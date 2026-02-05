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
    
    @abstractmethod
    def _declare_sensor_specific_parameters(self):
        """Declare parameters specific to the sensor type. Must be implemented by subclass."""
        pass
    
    @abstractmethod
    def _apply_sensor_specific_fault(self, msg: Any, fault_type: int) -> Any:
        """Apply sensor-specific fault. Must be implemented by subclass."""
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
        self.get_logger().debug('Fault state reset')
    
    def get_current_time(self) -> float:
        """Get current time in seconds"""
        return self.get_clock().now().nanoseconds / 1e9
    
    def get_elapsed_time(self) -> float:
        """Get elapsed time since first message"""
        if self.start_time is None:
            return 0.0
        return self.get_current_time() - self.start_time
    
    def process_message(self, msg: Any, fault_type_enum: int) -> Any:
        """
        Common message processing logic.
        
        Args:
            msg: Original sensor message
            fault_type_enum: Integer fault type value
            
        Returns:
            Processed message (with or without fault)
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
            return msg
        
        # Apply the sensor-specific fault
        return self._apply_sensor_specific_fault(msg, fault_type_enum)
    
    # ================================================================
    # Common Fault Implementations
    # ================================================================
    
    def apply_stuck_fault(self, msg: Any, update_header_func=None) -> Any:
        """
        Apply stuck-at-fault (common to all sensors).
        
        Simulates hardware failure where:
        - Timestamp continues to update
        - Sensor values remain frozen
        
        Args:
            msg: Current sensor message
            update_header_func: Optional function to update header timestamp
            
        Returns:
            Stuck message or original message
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
                return msg
            
            # Return stuck message with updated timestamp
            faulty_msg = copy.deepcopy(self.stuck_message)
            if update_header_func:
                faulty_msg = update_header_func(faulty_msg, msg)
            elif hasattr(faulty_msg, 'header'):
                faulty_msg.header.stamp = msg.header.stamp
            return faulty_msg
        
        return msg
    
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
            Tuple of (faulty_msg, is_dropout)
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
        
        return msg, False