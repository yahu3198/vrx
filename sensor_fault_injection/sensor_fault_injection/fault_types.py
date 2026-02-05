#!/usr/bin/env python3
"""
Fault Type Definitions for Sensor Fault Injection

This module defines the fault types for GPS and IMU sensors.
"""

from enum import IntEnum


class GPSFaultType(IntEnum):
    """Enumeration of GPS fault types"""
    NONE = 0
    OUTAGE = 1          # Complete signal loss / dropout
    JUMP = 2            # Instantaneous position offset
    STUCK = 3           # Frozen sensor readings
    MULTIPATH = 4       # Oscillating correlated errors


class IMUFaultType(IntEnum):
    """Enumeration of IMU fault types"""
    NONE = 0
    STUCK = 1           # Frozen sensor readings
    SATURATION = 2      # Output clipped at max/min values


# Fault type descriptions for logging and documentation
GPS_FAULT_DESCRIPTIONS = {
    GPSFaultType.NONE: "No fault (pass-through)",
    GPSFaultType.OUTAGE: "Complete signal loss / dropout",
    GPSFaultType.JUMP: "Instantaneous position offset",
    GPSFaultType.STUCK: "Frozen sensor readings",
    GPSFaultType.MULTIPATH: "Oscillating correlated errors",
}

IMU_FAULT_DESCRIPTIONS = {
    IMUFaultType.NONE: "No fault (pass-through)",
    IMUFaultType.STUCK: "Frozen sensor readings",
    IMUFaultType.SATURATION: "Output clipped at max/min values",
}