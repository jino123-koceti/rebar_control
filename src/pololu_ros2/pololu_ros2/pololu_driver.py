#!/usr/bin/env python3
"""Pololu driver module for motor controllers using pyserial

This module handles the lower level logic of writing to a pololu motor
controller using python.

Example:
    Here is an example of usage. The first Daisy object sets the port and
    all the of the rest use the same port.
        motor0 = Daisy(0, port="/dev/ttyUSB0")  # only first one has to be set
        motor1 = Daisy(1)  # all following will use the first port
        motor2 = Daisy(2)

        motor1.forward(1600)  # drive forward the motor with device number 1

        # All motors are using port "/dev/ttyUSB0" to send commands
        # Each device is addressed by their device number
"""

from __future__ import division
import serial

BAUD_SYNC = b'\x80'  # Required to sync the baud rate for older devices
PROTOCOL = b'\xAA'   # First part of command to write using Pololu Protocol
FORWARD = b'\x05'    # Drive motor forward command
BACKWARD = b'\x06'   # Drive motor reverse command
START = b'\x03'      # Exit Safe-Start mode so the motor can run
STOP = b'\x60'       # Stops the motor and enters Safe-Start mode

# Compact Protocol commands (no device ID needed)
COMPACT_FORWARD = b'\x85'   # Drive motor forward (Compact)
COMPACT_BACKWARD = b'\x86'  # Drive motor reverse (Compact)
COMPACT_START = b'\x83'     # Exit Safe-Start (Compact)
COMPACT_STOP = b'\xE0'      # Stop motor (Compact)


class Daisy(object):
    """Represents a single Pololu Simple Motor Controller in a daisy chain

    This class offers an interface to daisy chain several Pololu Simple
    Motor Controllers. The daisy chained modules all share the same serial
    connection. To target specific devices, every command is sent with the
    device number of the device. You must configure the devices to have
    different numbers. This must be done using the Simple Motor Controller
    setup software (https://www.pololu.com/docs/0J44/3).

    Initialize every separate board you want to talk to with its
    set device number.

    Attributes:
        dev_num (bytes): Device number of Pololu board to be commanded
    """

    count = 0  # static variable to keep track of number of Daisys open
    ser = None  # initialize static variable ser to None

    def __init__(self, dev_num, port="/dev/ttyUSB0", baudrate=9600, use_compact=True):
        """Set motor id and open serial connection if not already open

        Args:
            dev_num: Device number (0-127) for daisy chaining
            port: Serial port
            baudrate: Baud rate
            use_compact: Use Compact Protocol (True) or Pololu Protocol (False)
        """

        if dev_num < 0 or dev_num > 127:
            raise Exception("Invalid motor id, must set to id of motor (0-127) for daisy chaining")

        Daisy.count += 1  # increment count of controllers
        self.dev_num = bytes([dev_num])  # set device number to use in other commands
        self.use_compact = use_compact  # protocol mode

        # if serial connection has not been made yet
        if Daisy.ser is None or not Daisy.ser.isOpen():
            Daisy.ser = serial.Serial(port, baudrate=baudrate, timeout=1)
            Daisy.ser.write(BAUD_SYNC)  # sync old devices by writing 0x80

        self._exit_safe_start()  # make it so pololu reacts to commands

    def __del__(self):
        """Decrement count, stop motor, and close port if it's the last connection"""
        Daisy.count -= 1  # decrement count of controllers
        self._stop_motor()  # safely stop current motor
        
        # if this is the last controller open
        if Daisy.count <= 0:
            if Daisy.ser is not None and Daisy.ser.isOpen():
                Daisy.ser.close()
                print("Serial connection closed")

    def _send_command(self, command, compact_command, databyte3, databyte4):
        """Sends a two-byte command using the appropriate protocol."""
        if self.use_compact:
            cmd = compact_command + bytes([databyte3, databyte4])
            print(f"DEBUG: Sending compact command: {cmd.hex()} (data3={databyte3}, data4={databyte4})")
        else:
            cmd = PROTOCOL + self.dev_num + command + bytes([databyte3, databyte4])
            print(f"DEBUG: Sending command: {cmd.hex()} (dev={self.dev_num[0]}, cmd={command.hex()}, data3={databyte3}, data4={databyte4})")
        Daisy.ser.write(cmd)
        Daisy.ser.flush()  # Ensure data is sent immediately

    def _send_command_single(self, command, compact_command):
        """Sends a one-byte command using the appropriate protocol."""
        if self.use_compact:
            cmd = compact_command
            print(f"DEBUG: Sending compact single command: {cmd.hex()}")
        else:
            cmd = PROTOCOL + self.dev_num + command
            print(f"DEBUG: Sending single command: {cmd.hex()} (dev={self.dev_num[0]}, cmd={command.hex()})")
        Daisy.ser.write(cmd)
        Daisy.ser.flush()  # Ensure data is sent immediately

    def _exit_safe_start(self):
        """Exit safe start so you can freely send commands to Pololu.
        This must be run before run other commands."""
        self._send_command_single(START, COMPACT_START)

    def _stop_motor(self):
        """Immediately stops the motor and enters safe start mode"""
        self._send_command_single(STOP, COMPACT_STOP)

    # USER METHODS

    def forward(self, speed):
        """Drive motor forward at specified speed (0 to 3200)"""
        speed = max(min(3200, speed), 0)  # enforce bounds
        self._exit_safe_start()

        # Pololu Protocol uses 5-bit + 7-bit format for 12-bit data
        # databyte3 contains the lower 5 bits
        # databyte4 contains the upper 7 bits
        self._send_command(FORWARD, COMPACT_FORWARD, speed & 0x1F, (speed >> 5) & 0x7F)

    def backward(self, speed):
        """Drive motor backward (reverse) at specified speed (0 to 3200)"""
        speed = max(min(3200, speed), 0)  # enforce bounds
        self._exit_safe_start()
        self._send_command(BACKWARD, COMPACT_BACKWARD, speed & 0x1F, (speed >> 5) & 0x7F)

    def drive(self, speed):
        """Drive motor in direction based on speed (-3200, 3200)"""
        if speed < 0:
            self.backward(-speed)
        else:
            self.forward(speed)

    def stop(self):
        """Stop the motor"""
        self._stop_motor()
