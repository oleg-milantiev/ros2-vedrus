#!/usr/bin/env python3

import serial
import sys
import termios
import tty
import threading
import time

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# Add PID parameter tracking
pid_values = {'P': 0.0, 'I': 0.0, 'D': 0.0}

def get_key():
    """Get a single keypress from stdin without waiting for enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def read_serial(ser):
    """Read and print serial data in a separate thread"""
    while True:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8').strip()
                # Parse PID values from VEDL/VEDR format
                if line.startswith(('VEDL', 'VEDR')):
                    # Split the line into sections
                    sections = line.split(',')
                    for section in sections:
                        if section.startswith('PID:'):
                            # Extract PID values (format: PID:P/I/D)
                            pid_str = section.split(':')[1]
                            p, i, d = map(float, pid_str.split('/'))
                            pid_values['P'] = p
                            pid_values['I'] = i
                            pid_values['D'] = d
                print(f"{line}     \r") 
            except UnicodeDecodeError:
                pass
            except serial.SerialException:
                print("\nSerial connection lost!")
                break

def main():
    try:
        # Open serial port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}")
        
        # Start reading thread
        read_thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
        read_thread.start()
        
        print("Commands:")
        print("S=Stop, 0-9=Speed")
        print("P/p=Inc/Dec Kp, I/i=Inc/Dec Ki, D/d=Inc/Dec Kd")
        print("Q=Quit")
        print("\nEnter command: ", end='', flush=True)
        
        while True:
            key = get_key()
            
            if key.lower() == 'q':
                break
            elif key.lower() == 's':
                ser.write(b'S\n')
                print("\rSending: STOP")
            elif key.isdigit():
                speed = int(key) * 5
                command = f'M{speed}\n'.encode()
                ser.write(command)
                print(f"\rSending: MOVE {speed}")
            elif key in ['P', 'p', 'I', 'i', 'D', 'd']:
                param = key.upper()
                increment = 0.1 if key.isupper() else -0.1
                new_value = max(0.0, pid_values[param] + increment)  # Prevent negative values
                if new_value != pid_values[param]:  # Only update if value changed
                    pid_values[param] = new_value
                    command = f'{param}{increment}\n'.encode()
                    ser.write(command)
                    print(f"\rSending: Adjust {param} by {increment} (Current value: {pid_values[param]:.2f})")
                else:
                    print(f"\rCannot decrease {param} below 0.0 (Current value: {pid_values[param]:.2f})")
                
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'ser' in locals():
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main() 