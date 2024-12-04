import serial
import matplotlib.pyplot as plt
from collections import deque
import threading
import time

# Serial port configuration
SERIAL_PORT = "COM6"  # Replace with your serial port (e.g., COM3, /dev/ttyUSB0)
BAUD_RATE = 115200

# Data storage
max_points = 100
input_data = deque([0] * max_points, maxlen=max_points)
output1_data = deque([0] * max_points, maxlen=max_points)
output2_data = deque([0] * max_points, maxlen=max_points)
time_data = deque([0] * max_points, maxlen=max_points)

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
time.sleep(2)  # Wait for the connection to establish

# Function to send commands to the Arduino
def send_command():
    while True:
        command = input("Enter command (e.g., Kp2.5, Ki0.5, Kd1.0, setpoint0.0): ")
        ser.write((command + "\n").encode("utf-8"))
        time.sleep(0.1)  # Small delay to avoid overwhelming the serial port

# Start a thread for sending commands
command_thread = threading.Thread(target=send_command)
command_thread.daemon = True
command_thread.start()

# Initialize plot
plt.ion()
fig, ax = plt.subplots()
input_line, = ax.plot([], [], label="Input (Pitch)")
output1_line, = ax.plot([], [], label="Output1 (Motor1)")
output2_line, = ax.plot([], [], label="Output2 (Motor2)")
ax.set_xlim(0, max_points)
ax.set_ylim(-100, 100)  # Adjust based on your system's range
ax.legend()
ax.set_title("PID Controller Tuning")
ax.set_xlabel("Time")
ax.set_ylabel("Value")

try:
    start_time = time.time()
    while True:
        # Read data from serial
        line = ser.readline().decode("utf-8").strip()
        if "Input:" in line and "Output1:" in line and "Output2:" in line:
            # Parse the data
            parts = line.split(", ")
            input_value = float(parts[0].split(": ")[1])
            output1_value = float(parts[1].split(": ")[1])
            output2_value = float(parts[2].split(": ")[1])

            # Update data
            current_time = time.time() - start_time
            time_data.append(current_time)
            input_data.append(input_value)
            output1_data.append(output1_value)
            output2_data.append(output2_value)

            # Update plot
            input_line.set_data(range(len(input_data)), input_data)
            output1_line.set_data(range(len(output1_data)), output1_data)
            output2_line.set_data(range(len(output2_data)), output2_data)
            ax.set_xlim(0, len(input_data))
            ax.set_ylim(min(min(input_data), min(output1_data), min(output2_data)) - 10,
                        max(max(input_data), max(output1_data), max(output2_data)) + 10)
            plt.pause(0.01)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    plt.ioff()
    plt.show()