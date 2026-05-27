#!/usr/bin/env python3

import serial
import time
import sys

# Roboteq commands
PORT = '/dev/ttyACM0'
BAUD = 115200
TIMEOUT = 1.0

def send_command(ser, command):
    """Send command and read response"""
    ser.write((command + '\r').encode())
    time.sleep(0.1)
    response = ser.read(100).decode('utf-8', errors='ignore')
    return response.strip()

def main():
    try:
        # Open serial port
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        print(f"Connected to {PORT} at {BAUD} baud")
        time.sleep(2)  # Wait for connection to stabilize
        
        # Turn echo off
        print("Turning echo off...")
        send_command(ser, "^ECHOF 1")
        
        # Example: Move forward (both motors at 100 RPM)
        print("\nSending forward command (motor 1: 100 RPM, motor 2: 100 RPM)...")
        response1 = send_command(ser, "!S 1 100")
        print(f"Motor 1 response: {response1}")
        response2 = send_command(ser, "!S 2 100")
        print(f"Motor 2 response: {response2}")
        
        time.sleep(2)
        
        # Stop motors
        print("\nStopping motors...")
        send_command(ser, "!S 1 0")
        send_command(ser, "!S 2 0")
        
        # Read current RPM
        print("\nReading current RPM...")
        rpm_response = send_command(ser, "?S")
        print(f"RPM response: {rpm_response}")
        
        ser.close()
        print("\nDisconnected")
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print(f"Make sure {PORT} exists and you have permissions")
    except KeyboardInterrupt:
        print("\nStopping motors...")
        try:
            send_command(ser, "!S 1 0")
            send_command(ser, "!S 2 0")
            ser.close()
        except:
            pass
        print("Exited")

if __name__ == '__main__':
    if len(sys.argv) > 1:
        # Custom command: python test_roboteq.py "!S 1 50"
        port = '/dev/ttyACM0'
        baud = 115200
        ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2)
        command = sys.argv[1]
        response = send_command(ser, command)
        print(f"Command: {command}")
        print(f"Response: {response}")
        ser.close()
    else:
        main()
