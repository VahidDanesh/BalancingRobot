import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re

# Configuration
SERIAL_PORT = "/dev/ttyUSB0"  # Replace with your serial port (e.g., "COM3" on Windows, "/dev/ttyUSB0" on Linux/Mac)
BAUD_RATE = 115200
PLOT_INTERVAL = 100  # Update interval in milliseconds
MAX_POINTS = 200  # Maximum points to display on the plot

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Data buffers
data = {
    "Input Pos": [],
    "Output Pos": [],
    "Input Speed": [],
    "Output Speed": [],
    "Input Angle": [],
    "Output Angle": [],
}

time = []

# Plot setup
fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
plots = {}

for i, key in enumerate(data.keys()):
    ax_idx = i // 2
    if key not in plots:
        plots[key], = ax[ax_idx].plot([], [], label=key)
    ax[ax_idx].set_ylabel(key.split()[0])  # Label based on the type (Pos, Speed, Angle)
    ax[ax_idx].legend(loc="upper right")
ax[-1].set_xlabel("Time (s)")

# Function to parse serial data
def parse_serial_data(line):
    try:
        match = re.findall(r"Input Pos: ([\d.-]+) Output Pos: ([\d.-]+) Input Speed: ([\d.-]+) Output Speed: ([\d.-]+) Input angle: ([\d.-]+) Output angle: ([\d.-]+)", line)
        if match:
            return list(map(float, match[0]))
    except Exception as e:
        print(f"Error parsing line: {line} -> {e}")
    return None

# Update function for animation
def update(frame):
    global time
    line = ser.readline().decode("utf-8").strip()
    parsed_data = parse_serial_data(line)
    if parsed_data:
        t = len(time) * (PLOT_INTERVAL / 1000)  # Calculate time in seconds
        time.append(t)
        for idx, key in enumerate(data.keys()):
            data[key].append(parsed_data[idx])
            data[key] = data[key][-MAX_POINTS:]  # Keep only the latest points

        for key, plot in plots.items():
            plot.set_data(time[-MAX_POINTS:], data[key])

        for axis in ax:
            axis.relim()
            axis.autoscale_view()

# Initialize plot
def init():
    for plot in plots.values():
        plot.set_data([], [])
    return plots.values()

# Run the animation
ani = FuncAnimation(fig, update, init_func=init, interval=PLOT_INTERVAL, blit=False)
plt.tight_layout()
plt.show()

# Close the serial connection on exit
ser.close()
