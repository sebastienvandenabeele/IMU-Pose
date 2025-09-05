#!/usr/bin/env python3

import serial
import serial.tools.list_ports
import csv
import re
import time
from datetime import datetime

def find_pico_port():
    """Find the Raspberry Pi Pico serial port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'usbmodem' in port.device or 'ACM' in port.device:
            return port.device
    return None

def log_imu_data(duration_seconds=60, filename=None):
    """Log IMU data to CSV file"""
    
    port = find_pico_port()
    if not port:
        print("No Pico device found!")
        return
    
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"imu_data_{timestamp}.csv"
    
    print(f"Logging IMU data from {port}")
    print(f"Output file: {filename}")
    print(f"Duration: {duration_seconds} seconds")
    print("Press Ctrl+C to stop early\n")
    
    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'data_type', 'accel_x', 'accel_y', 'accel_z', 
                             'quat_real', 'quat_i', 'quat_j', 'quat_k', 'quat_accuracy']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                start_time = time.time()
                
                while (time.time() - start_time) < duration_seconds:
                    try:
                        if ser.in_waiting > 0:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                print(line)  # Show data being logged
                                
                                current_time = time.time()
                                row = {'timestamp': current_time}
                                
                                # Parse linear acceleration
                                accel_match = re.search(r'Linear Accel: ([-\d.]+), ([-\d.]+), ([-\d.]+)', line)
                                if accel_match:
                                    x, y, z = map(float, accel_match.groups())
                                    row.update({
                                        'data_type': 'acceleration',
                                        'accel_x': x,
                                        'accel_y': y,
                                        'accel_z': z
                                    })
                                    writer.writerow(row)
                                
                                # Parse quaternion
                                quat_match = re.search(r'Quaternion: real=([-\d.]+), i=([-\d.]+), j=([-\d.]+), k=([-\d.]+) \(accuracy=([-\d.]+)\)', line)
                                if quat_match:
                                    real, i, j, k, accuracy = map(float, quat_match.groups())
                                    row.update({
                                        'data_type': 'quaternion',
                                        'quat_real': real,
                                        'quat_i': i,
                                        'quat_j': j,
                                        'quat_k': k,
                                        'quat_accuracy': accuracy
                                    })
                                    writer.writerow(row)
                        
                        time.sleep(0.01)
                        
                    except KeyboardInterrupt:
                        print("\nStopping data logging...")
                        break
                        
    except serial.SerialException as e:
        print(f"Could not open serial port {port}: {e}")
    
    print(f"\nData saved to: {filename}")

def main():
    import sys
    
    print("BNO085 IMU Data Logger")
    print("======================")
    
    duration = 60  # Default 60 seconds
    filename = None
    
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            print("Invalid duration. Using default 60 seconds.")
    
    if len(sys.argv) > 2:
        filename = sys.argv[2]
    
    print(f"Usage: {sys.argv[0]} [duration_seconds] [filename.csv]")
    print()
    
    log_imu_data(duration, filename)

if __name__ == "__main__":
    main()
