import serial
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import tkinter as tk
from tkinter import ttk
import json

class PIDPlotter:
    def __init__(self, port='COM3', baudrate=115200, max_points=100):
        # Serial communication
        self.serial_port = serial.Serial(port, baudrate)
        time.sleep(2)  # Wait for Arduino to reset

        # Data storage
        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.inputs = deque(maxlen=max_points)
        self.outputs1 = deque(maxlen=max_points)
        self.outputs2 = deque(maxlen=max_points)
        self.setpoints = deque(maxlen=max_points)

        # Plotting setup
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.lines = {
            'input': self.ax.plot([], [], 'b-', label='Input (Angle)')[0],
            'output1': self.ax.plot([], [], 'r-', label='Output1')[0],
            'output2': self.ax.plot([], [], 'g-', label='Output2')[0],
            'setpoint': self.ax.plot([], [], 'k--', label='Setpoint')[0]
        }

        # Plot configuration
        self.ax.set_ylim(-45, 45)
        self.ax.set_xlim(0, 30)
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_title('PID Controller Response')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Angle (degrees) / Output')

        # Start time
        self.start_time = time.time()

        # Create control window
        self.create_control_window()

        # Animation
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=50,
            blit=True, cache_frame_data=False
        )

    def create_control_window(self):
        self.control_window = tk.Tk()
        self.control_window.title("PID Controller Parameters")

        # PID Parameters
        parameters = [
            ("Kp", "Kp"),
            ("Ki", "Ki"),
            ("Kd", "Kd"),
            ("Setpoint", "setpoint")
        ]

        for param, cmd in parameters:
            frame = ttk.Frame(self.control_window)
            frame.pack(padx=5, pady=5, fill="x")

            ttk.Label(frame, text=f"{param}:").pack(side="left")
            entry = ttk.Entry(frame, width=10)
            entry.pack(side="left", padx=5)

            ttk.Button(
                frame, 
                text=f"Update {param}",
                command=lambda c=cmd, e=entry: self.send_command(c, e.get())
            ).pack(side="left")

        # Save/Load buttons
        ttk.Button(
            self.control_window,
            text="Save Parameters",
            command=self.save_parameters
        ).pack(pady=5)

        ttk.Button(
            self.control_window,
            text="Load Parameters",
            command=self.load_parameters
        ).pack(pady=5)

    def send_command(self, command, value):
        try:
            value = float(value)
            command_str = f"{command}{value}\n"
            self.serial_port.write(command_str.encode())
        except ValueError:
            print(f"Invalid value for {command}")

    def save_parameters(self):
        params = {
            "Kp": float(self.control_window.children['!frame'].children['!entry'].get()),
            "Ki": float(self.control_window.children['!frame2'].children['!entry'].get()),
            "Kd": float(self.control_window.children['!frame3'].children['!entry'].get()),
            "setpoint": float(self.control_window.children['!frame4'].children['!entry'].get())
        }

        with open('pid_parameters.json', 'w') as f:
            json.dump(params, f)

    def load_parameters(self):
        try:
            with open('pid_parameters.json', 'r') as f:
                params = json.load(f)

            for param, value in params.items():
                self.send_command(param, str(value))

        except FileNotFoundError:
            print("No saved parameters found")

    def update_plot(self, frame):
        # Read serial data
        if self.serial_port.in_waiting:
            try:
                line = self.serial_port.readline().decode().strip()
                # Parse the data
                data = {}
                for item in line.split(','):
                    key, value = item.split(':')
                    data[key.strip()] = float(value.strip())

                # Update data
                current_time = time.time() - self.start_time
                self.times.append(current_time)
                self.inputs.append(data['Input'])
                self.outputs1.append(data['Output1'])
                self.outputs2.append(data['Output2'])
                self.setpoints.append(0)  # or whatever your setpoint is

            except (ValueError, KeyError) as e:
                print(f"Error parsing data: {e}")
                return self.lines.values()

        # Update plot data
        for line, data in zip(
            self.lines.values(),
            [self.inputs, self.outputs1, self.outputs2, self.setpoints]
        ):
            line.set_data(list(self.times), list(data))

        # Adjust plot limits if necessary
        if self.times:
            self.ax.set_xlim(max(0, self.times[-1] - 30), self.times[-1] + 2)

        return self.lines.values()

    def run(self):
        plt.show()

    def cleanup(self):
        self.serial_port.close()
        plt.close()

if __name__ == "__main__":
    # Create and run plotter
    plotter = PIDPlotter(port='COM3')  # Change COM port as needed
    try:
        plotter.run()
    finally:
        plotter.cleanup()

# Created/Modified files during execution:
# - pid_parameters.json (when saving parameters)