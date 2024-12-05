import serial
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import json

class PIDPlotter:
    def __init__(self, port='COM3', baudrate=115200, max_points=100):
        # Create main window
        self.root = tk.Tk()
        self.root.title("PID Controller Interface")

        # Create frames
        self.control_frame = ttk.Frame(self.root)
        self.control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)

        self.plot_frame = ttk.Frame(self.root)
        self.plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Serial communication
        try:
            self.serial_port = serial.Serial(port, baudrate)
            time.sleep(2)  # Wait for Arduino to reset
        except serial.SerialException:
            print(f"Could not open port {port}")
            self.serial_port = None

        # Data storage
        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.inputs = deque(maxlen=max_points)
        self.outputs1 = deque(maxlen=max_points)
        self.outputs2 = deque(maxlen=max_points)
        self.setpoints = deque(maxlen=max_points)

        # Create plot
        self.create_plot()

        # Create controls
        self.create_controls()

        # Start time
        self.start_time = time.time()

        # Add current PID values storage
        self.current_pid = {
            'Kp': 2.0,
            'Ki': 0.0,
            'Kd': 0.0
        }

        # Add labels for current values
        self.current_value_labels = {}

    def create_plot(self):
        # Create figure and axis
        self.fig, self.ax = plt.subplots(figsize=(10, 6))

        # Create lines
        self.lines = {
            'input': self.ax.plot([], [], 'b-', label='Input (Angle)')[0],
            'output1': self.ax.plot([], [], 'r-', label='Output1')[0],
            'output2': self.ax.plot([], [], 'g-', label='Output2')[0],
            'setpoint': self.ax.plot([], [], 'k--', label='Setpoint')[0]
        }

        # Configure plot
        self.ax.set_ylim(-45, 45)
        self.ax.set_xlim(0, 30)
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_title('PID Controller Response')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Angle (degrees) / Output')

        # Embed plot in tkinter window
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_controls(self):
        # Title for control panel
        ttk.Label(self.control_frame, text="PID Parameters", font=('Arial', 12, 'bold')).pack(pady=5)

        # Create entry variables
        self.pid_vars = {
            'Kp': tk.StringVar(value="2.0"),
            'Ki': tk.StringVar(value="0.0"),
            'Kd': tk.StringVar(value="0.0"),
            'setpoint': tk.StringVar(value="0.0")
        }

        # Create parameter controls
        for param, var in self.pid_vars.items():
            frame = ttk.LabelFrame(self.control_frame, text=param)
            frame.pack(padx=5, pady=5, fill="x")

            entry = ttk.Entry(frame, textvariable=var, width=10)
            entry.pack(side="left", padx=5, pady=5)

            ttk.Button(
                frame,
                text="Update",
                command=lambda p=param, v=var: self.send_command(p, v.get())
            ).pack(side="left", padx=5, pady=5)

        # Add save/load buttons
        ttk.Button(
            self.control_frame,
            text="Save Parameters",
            command=self.save_parameters
        ).pack(pady=5, fill="x", padx=5)

        ttk.Button(
            self.control_frame,
            text="Load Parameters",
            command=self.load_parameters
        ).pack(pady=5, fill="x", padx=5)

        # Add status display
        self.status_var = tk.StringVar(value="Status: Ready")
        ttk.Label(self.control_frame, textvariable=self.status_var).pack(pady=5)

    def send_command(self, command, value):
        if self.serial_port is None:
            self.status_var.set("Status: No serial connection")
            return

        try:
            value = float(value)
            command_str = f"{command}{value}\n"
            self.serial_port.write(command_str.encode())
            self.status_var.set(f"Status: Sent {command}={value}")
        except ValueError:
            self.status_var.set(f"Status: Invalid value for {command}")

    def save_parameters(self):
        params = {param: float(var.get()) for param, var in self.pid_vars.items()}
        try:
            with open('pid_parameters.json', 'w') as f:
                json.dump(params, f)
            self.status_var.set("Status: Parameters saved")
        except Exception as e:
            self.status_var.set(f"Status: Error saving parameters - {str(e)}")

    def load_parameters(self):
        try:
            with open('pid_parameters.json', 'r') as f:
                params = json.load(f)
            for param, value in params.items():
                self.pid_vars[param].set(str(value))
                self.send_command(param, str(value))
            self.status_var.set("Status: Parameters loaded")
        except FileNotFoundError:
            self.status_var.set("Status: No saved parameters found")
        except Exception as e:
            self.status_var.set(f"Status: Error loading parameters - {str(e)}")

    def update_plot(self):
        if self.serial_port and self.serial_port.in_waiting:
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
                self.setpoints.append(float(self.pid_vars['setpoint'].get()))

                # Update plot data
                for line, data in zip(
                    self.lines.values(),
                    [self.inputs, self.outputs1, self.outputs2, self.setpoints]
                ):
                    line.set_data(list(self.times), list(data))

                # Adjust plot limits if necessary
                if self.times:
                    self.ax.set_xlim(max(0, self.times[-1] - 30), self.times[-1] + 2)

                self.canvas.draw()

            except Exception as e:
                print(f"Error updating plot: {e}")

        # Schedule next update
        self.root.after(50, self.update_plot)

    def run(self):
        # Start plot updates
        self.root.after(50, self.update_plot)
        # Start main loop
        self.root.mainloop()

    def cleanup(self):
        if self.serial_port:
            self.serial_port.close()

if __name__ == "__main__":
    # Create and run plotter
    plotter = PIDPlotter(port='/dev/ttyUSB0')  # Change COM port as needed
    try:
        plotter.run()
    finally:
        plotter.cleanup()

# Created/Modified files during execution:
# - pid_parameters.json (when saving parameters)