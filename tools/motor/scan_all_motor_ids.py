#!/usr/bin/env python3
"""
Scan extended CAN ID range for RMD motors
Tests IDs from 0x140 to 0x14F
"""

import socket
import struct
import time
import subprocess

class ExtendedMotorScanner:
    def __init__(self, can_interface='can2'):
        self.can_interface = can_interface
        self.socket = None
        # Scan all possible RMD motor IDs
        self.motor_ids = list(range(0x140, 0x150))  # 0x140 to 0x14F
        self.READ_STATUS = 0x9A

    def check_can_status(self):
        """Check CAN interface status"""
        try:
            result = subprocess.run(['ip', '-details', 'link', 'show', self.can_interface], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print(f"CAN interface status: {self.can_interface}")
                print(result.stdout)
            else:
                print(f"⚠ Could not check CAN interface status")
        except Exception as e:
            print(f"⚠ Error checking CAN status: {e}")

    def setup(self):
        try:
            self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.socket.bind((self.can_interface,))
            self.socket.settimeout(0.3)
            print(f"✓ Connected to {self.can_interface}")
            self.check_can_status()
            print()
            return True
        except Exception as e:
            print(f"✗ Failed: {e}")
            return False

    def send_can_frame(self, can_id, data):
        can_dlc = len(data)
        frame = struct.pack("IB3x8s", can_id, can_dlc, data.ljust(8, b'\x00'))
        
        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.socket.send(frame)
                return  # Success
            except OSError as e:
                if e.errno == 105:  # No buffer space available
                    wait_time = 0.2 * (attempt + 1)  # Increasing wait time
                    print(f"⚠ CAN buffer full, waiting {wait_time:.1f}s...")
                    time.sleep(wait_time)
                    if attempt == max_retries - 1:
                        print(f"✗ Failed to send after {max_retries} attempts")
                        raise
                else:
                    raise

    def recv_can_frame(self, timeout=0.3):
        try:
            self.socket.settimeout(timeout)
            frame = self.socket.recv(16)
            can_id = struct.unpack("I", frame[:4])[0] & 0x1FFFFFFF
            dlc = frame[4]
            data = frame[8:8+dlc]
            return can_id, data
        except socket.timeout:
            return None, None

    def scan_all_ids(self):
        print("="*70)
        print("EXTENDED MOTOR ID SCAN")
        print("="*70)
        print(f"Scanning CAN IDs: 0x140 - 0x14F")
        print("="*70 + "\n")

        detected = []

        for motor_id in self.motor_ids:
            print(f"Scanning ID 0x{motor_id:03X}...", end=" ")
            
            # Send read status command
            cmd_data = bytes([self.READ_STATUS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            self.send_can_frame(motor_id, cmd_data)
            
            # Give more time for transmission and response
            time.sleep(0.1)

            # Check for response
            can_id, data = self.recv_can_frame(timeout=0.3)

            if can_id is not None:
                detected.append({
                    'cmd_id': motor_id,
                    'resp_id': can_id,
                    'data': data
                })
                print(f"✓ Motor found!")
                print(f"  Command ID: 0x{motor_id:03X}")
                print(f"  Response ID: 0x{can_id:03X}")
                print(f"  Data: {data.hex().upper()}")

                if len(data) >= 5:
                    temp = data[1]
                    voltage = struct.unpack('<H', data[3:5])[0] / 10.0
                    print(f"  Temperature: {temp}°C")
                    print(f"  Voltage: {voltage}V")
                print()
            else:
                print("No response")

            # Longer delay between scans to prevent buffer overflow
            time.sleep(0.5)

        print("="*70)
        print("SCAN COMPLETE")
        print("="*70)
        print(f"Total motors detected: {len(detected)}")

        if detected:
            print("\nDetected motors:")
            for i, motor in enumerate(detected, 1):
                print(f"  Motor {i}:")
                print(f"    Command ID: 0x{motor['cmd_id']:03X}")
                print(f"    Response ID: 0x{motor['resp_id']:03X}")
        else:
            print("\n⚠ No motors detected!")
            print("\nPossible reasons:")
            print("  1. Motors not powered (check power LED)")
            print("  2. CAN cable not connected properly (check H/L)")
            print("  3. CAN terminator resistor (120Ω) missing")
            print("  4. Wrong bitrate (current: 1Mbps)")
            print("  5. All motors share the same CAN ID and respond simultaneously")
            print("\nTroubleshooting:")
            print("  - Check CAN bus status: ip -details link show can2")
            print("  - Try motor reboot (power cycle)")
            print("  - Verify physical connections")
        print("="*70 + "\n")

    def close(self):
        if self.socket:
            self.socket.close()

if __name__ == "__main__":
    scanner = ExtendedMotorScanner('can2')
    if scanner.setup():
        try:
            scanner.scan_all_ids()
        finally:
            scanner.close()
