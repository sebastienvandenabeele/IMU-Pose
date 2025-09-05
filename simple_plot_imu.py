#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import re
import time
import subprocess
import threading
import queue
from collections import deque

class SimpleIMUPlotter:
    def __init__(self, device='/dev/tty.usbmodem21301', max_points=500):
        self.device = device
        self.max_points = max_points
        
        # Data storage
        self.times = deque(maxlen=max_points)
        
        # Linear acceleration data
        self.accel_x = deque(maxlen=max_points)
        self.accel_y = deque(maxlen=max_points)
        self.accel_z = deque(maxlen=max_points)
        
        # Quaternion data
        self.quat_real = deque(maxlen=max_points)
        self.quat_i = deque(maxlen=max_points)
        self.quat_j = deque(maxlen=max_points)
        self.quat_k = deque(maxlen=max_points)
        self.quat_accuracy = deque(maxlen=max_points)
        
        # Data queue for thread communication
        self.data_queue = queue.Queue()
        self.running = False
        
        # Start time for relative timestamps
        self.start_time = time.time()
        
        # Setup plots
        self.setup_plots()
        
    def setup_plots(self):
        """Setup matplotlib subplots"""
        plt.style.use('default')
        
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('BNO085 IMU Data - Real-time Plotting', fontsize=16, fontweight='bold')
        
        # Linear Acceleration subplot
        self.ax1.set_title('Linear Acceleration', fontweight='bold')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Acceleration (m/s²)')
        self.ax1.grid(True, alpha=0.3)
        self.line_ax, = self.ax1.plot([], [], 'r-', label='X', linewidth=2)
        self.line_ay, = self.ax1.plot([], [], 'g-', label='Y', linewidth=2)
        self.line_az, = self.ax1.plot([], [], 'b-', label='Z', linewidth=2)
        self.ax1.legend()
        
        # Quaternion Real/Accuracy subplot
        self.ax2.set_title('Quaternion Real & Accuracy', fontweight='bold')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Value')
        self.ax2.grid(True, alpha=0.3)
        self.line_qr, = self.ax2.plot([], [], 'purple', label='Real', linewidth=2)
        self.line_qa, = self.ax2.plot([], [], 'orange', label='Accuracy', linewidth=2)
        self.ax2.legend()
        
        # Quaternion I,J,K subplot
        self.ax3.set_title('Quaternion I, J, K Components', fontweight='bold')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Value')
        self.ax3.grid(True, alpha=0.3)
        self.line_qi, = self.ax3.plot([], [], 'cyan', label='I', linewidth=2)
        self.line_qj, = self.ax3.plot([], [], 'magenta', label='J', linewidth=2)
        self.line_qk, = self.ax3.plot([], [], 'yellow', label='K', linewidth=2)
        self.ax3.legend()
        
        # 3D Acceleration magnitude and quaternion norm
        self.ax4.set_title('Derived Values', fontweight='bold')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Magnitude')
        self.ax4.grid(True, alpha=0.3)
        self.line_accel_mag, = self.ax4.plot([], [], 'red', label='Accel Magnitude', linewidth=2)
        self.line_quat_norm, = self.ax4.plot([], [], 'blue', label='Quaternion Norm', linewidth=2)
        self.ax4.legend()
        
        plt.tight_layout()
        
    def parse_serial_data(self, line):
        """Parse serial data lines and extract IMU values"""
        try:
            # Parse linear acceleration: "Linear Accel: -0.123, 0.456, 9.789 m/s²"
            accel_match = re.search(r'Linear Accel: ([-\d.]+), ([-\d.]+), ([-\d.]+)', line)
            if accel_match:
                x, y, z = map(float, accel_match.groups())
                current_time = time.time() - self.start_time
                self.data_queue.put(('accel', current_time, x, y, z))
                return
            
            # Parse quaternion: "Quaternion: real=0.998, i=0.012, j=-0.034, k=0.056 (accuracy=3.000)"
            quat_match = re.search(r'Quaternion: real=([-\d.]+), i=([-\d.]+), j=([-\d.]+), k=([-\d.]+) \(accuracy=([-\d.]+)\)', line)
            if quat_match:
                real, i, j, k, accuracy = map(float, quat_match.groups())
                current_time = time.time() - self.start_time
                self.data_queue.put(('quat', current_time, real, i, j, k, accuracy))
                return
                
        except Exception as e:
            print(f"Error parsing line '{line}': {e}")
    
    def serial_reader(self):
        """Thread function to read serial data using cat command"""
        print(f"Connecting to {self.device}...")
        
        try:
            # Use subprocess to run cat command and read from the device
            # First set the baud rate
            subprocess.run(['stty', '-f', self.device, '115200'], check=True)
            
            # Start cat process to read from device
            process = subprocess.Popen(['cat', self.device], 
                                     stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE,
                                     universal_newlines=True,
                                     bufsize=1)
            
            print(f"Connected to {self.device}")
            
            while self.running and process.poll() is None:
                try:
                    line = process.stdout.readline()
                    if line:
                        line = line.strip()
                        if line:
                            print(f"Serial: {line}")  # Debug output
                            self.parse_serial_data(line)
                except Exception as e:
                    print(f"Error reading line: {e}")
                    
            process.terminate()
            
        except subprocess.CalledProcessError as e:
            print(f"Error setting up serial connection: {e}")
        except Exception as e:
            print(f"Serial connection error: {e}")
    
    def update_plot(self, frame):
        """Update plot with new data"""
        # Process all queued data
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                data_type = data[0]
                
                if data_type == 'accel':
                    _, t, x, y, z = data
                    self.times.append(t)
                    self.accel_x.append(x)
                    self.accel_y.append(y)
                    self.accel_z.append(z)
                    
                elif data_type == 'quat':
                    _, t, real, i, j, k, accuracy = data
                    # Only add quaternion data if we have time data
                    if self.times:
                        # Pad quaternion data to match time length if needed
                        while len(self.quat_real) < len(self.times) - 1:
                            self.quat_real.append(np.nan)
                            self.quat_i.append(np.nan)
                            self.quat_j.append(np.nan)
                            self.quat_k.append(np.nan)
                            self.quat_accuracy.append(np.nan)
                        
                        self.quat_real.append(real)
                        self.quat_i.append(i)
                        self.quat_j.append(j)
                        self.quat_k.append(k)
                        self.quat_accuracy.append(accuracy)
                        
            except queue.Empty:
                break
        
        # Update plots if we have data
        if len(self.times) > 0:
            times_array = np.array(self.times)
            
            # Update acceleration plot
            if len(self.accel_x) > 0:
                self.line_ax.set_data(times_array[:len(self.accel_x)], self.accel_x)
                self.line_ay.set_data(times_array[:len(self.accel_y)], self.accel_y)
                self.line_az.set_data(times_array[:len(self.accel_z)], self.accel_z)
                
                # Update acceleration magnitude
                accel_mag = [np.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(self.accel_x, self.accel_y, self.accel_z)]
                self.line_accel_mag.set_data(times_array[:len(accel_mag)], accel_mag)
            
            # Update quaternion plots
            if len(self.quat_real) > 0:
                quat_times = times_array[:len(self.quat_real)]
                self.line_qr.set_data(quat_times, self.quat_real)
                self.line_qa.set_data(quat_times, self.quat_accuracy)
                self.line_qi.set_data(quat_times, self.quat_i)
                self.line_qj.set_data(quat_times, self.quat_j)
                self.line_qk.set_data(quat_times, self.quat_k)
                
                # Update quaternion norm
                quat_norm = [np.sqrt(r**2 + i**2 + j**2 + k**2) for r, i, j, k in 
                           zip(self.quat_real, self.quat_i, self.quat_j, self.quat_k) if not np.isnan(r)]
                if quat_norm:
                    self.line_quat_norm.set_data(quat_times[:len(quat_norm)], quat_norm)
            
            # Auto-scale axes
            for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
                ax.relim()
                ax.autoscale_view()
        
        return (self.line_ax, self.line_ay, self.line_az, self.line_qr, self.line_qa, 
                self.line_qi, self.line_qj, self.line_qk, self.line_accel_mag, self.line_quat_norm)
    
    def start_plotting(self):
        """Start the plotting process"""
        self.running = True
        
        # Start serial reader thread
        serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
        serial_thread.start()
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.running = False

def main():
    import sys
    
    device = '/dev/tty.usbmodem21301'  # Default device
    
    if len(sys.argv) > 1:
        device = sys.argv[1]
    
    print("Simple BNO085 IMU Data Plotter")
    print("==============================")
    print(f"Listening to: {device}")
    print("Make sure your Pico is connected and running the BNO085 code.")
    print("Close the plot window or press Ctrl+C to exit.")
    print()
    
    plotter = SimpleIMUPlotter(device=device, max_points=1000)
    plotter.start_plotting()

if __name__ == "__main__":
    main()
