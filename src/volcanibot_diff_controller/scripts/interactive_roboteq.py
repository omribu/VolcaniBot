#!/usr/bin/env python3

import serial
import time
import sys

PORT = '/dev/ttyACM0'
BAUD = 115200
TIMEOUT = 1.0

def send_command(ser, command):
    """Send command and read response"""
    full_command = command + '\r'
    ser.write(full_command.encode())
    time.sleep(0.1)
    response = ser.read(100).decode('utf-8', errors='ignore')
    return response.strip()

def main():
    try:
        print(f"Connecting to {PORT} at {BAUD} baud...")
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        time.sleep(2)  # Wait for connection to stabilize
        print("Connected! Type commands (or 'quit' to exit)")
        print("Examples: !S 1 100, !S 2 100, ?S, !S 1 0")
        print("-" * 50)
        
        # Turn echo off
        send_command(ser, "^ECHOF 1")
        
        while True:
            try:
                command = input("Roboteq> ").strip()
                
                if command.lower() in ['quit', 'exit', 'q']:
                    break
                
                if not command:
                    continue
                
                # Send command
                response = send_command(ser, command)
                if response:
                    print(f"Response: {response}")
                else:
                    print("(No response)")
                    
            except KeyboardInterrupt:
                break
        
        # Stop motors before exiting
        print("\nStopping motors...")
        send_command(ser, "!S 1 0")
        send_command(ser, "!S 2 0")
        ser.close()
        print("Disconnected")
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        print(f"Make sure {PORT} exists and you have permissions")
        print("Try: sudo chmod 666 /dev/ttyACM0")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
