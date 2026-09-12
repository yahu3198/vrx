#!/usr/bin/env python3
"""
Fault Type Definitions for Sensor Fault Injection

This module defines the fault types for GPS, IMU, and LiDAR sensors.
"""

from enum import IntEnum


class GPSFaultType(IntEnum):
    """Enumeration of GPS fault types"""
    NONE = 0
    OUTAGE = 1          # Complete signal loss / dropout
    JUMP = 2            # Instantaneous position offset
    STUCK = 3           # Frozen sensor readings
    MULTIPATH = 4       # Oscillating correlated errors
    DEGRADED_RATE = 5   # Reduced update frequency


class IMUFaultType(IntEnum):
    """Enumeration of IMU fault types"""
    NONE = 0
    STUCK = 1           # Frozen sensor readings
    SATURATION = 2      # Output clipped at max/min values
    DEGRADED_RATE = 3   # Reduced update frequency


class LiDARFaultType(IntEnum):
    """Enumeration of LiDAR fault types"""
    NONE = 0
    INCREASED_NOISE = 1  # Noise amplitude increased (rain, fog, spray)
    REDUCED_RANGE = 2    # Max detection distance decreased
    STUCK = 3            # Frozen point cloud readings


# Fault type descriptions for logging and documentation
GPS_FAULT_DESCRIPTIONS = {
    GPSFaultType.NONE: "No fault (pass-through)",
    GPSFaultType.OUTAGE: "Complete signal loss / dropout",
    GPSFaultType.JUMP: "Instantaneous position offset",
    GPSFaultType.STUCK: "Frozen sensor readings",
    GPSFaultType.MULTIPATH: "Oscillating correlated errors",
    GPSFaultType.DEGRADED_RATE: "Reduced update frequency",
}

IMU_FAULT_DESCRIPTIONS = {
    IMUFaultType.NONE: "No fault (pass-through)",
    IMUFaultType.STUCK: "Frozen sensor readings",
    IMUFaultType.SATURATION: "Output clipped at max/min values",
    IMUFaultType.DEGRADED_RATE: "Reduced update frequency",
}

LIDAR_FAULT_DESCRIPTIONS = {
    LiDARFaultType.NONE: "No fault (pass-through)",
    LiDARFaultType.INCREASED_NOISE: "Noise amplitude increased (rain, fog, spray, dust)",
    LiDARFaultType.REDUCED_RANGE: "Max detection distance decreased (fog, rain, dirty lens)",
    LiDARFaultType.STUCK: "Frozen point cloud readings (hardware failure, firmware crash)",
}