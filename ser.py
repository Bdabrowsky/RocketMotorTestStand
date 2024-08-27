import serial
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

# Configure the serial port (adjust the parameters as needed)
ser = serial.Serial('/dev/cu.usbmodem1421201', 921600, timeout=None)  # Change 'COM3' to your serial port
csv_filename = 'thrust_data.csv'

# Prepare the CSV file
with open(csv_filename, 'a', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['Timestamp', 'Thrust'])  # Write the header

# Prepare for live plotting
plt.style.use('fivethirtyeight')
fig, ax = plt.subplots()
x_data = deque(maxlen=15000)  # Store only the last 15000 points for a fast update
y_data = deque(maxlen=15000)
line, = ax.plot([], [], lw=2)
ax.set_xlabel('Timestamp')
ax.set_ylabel('Thrust')
ax.set_title('Live Thrust Data')

def init():
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 400)
    return line,

def update_plot(frame):
    global x_data, y_data
    line.set_data(x_data, y_data)
    if len(x_data) > 0:
        ax.set_xlim(min(x_data), max(x_data))
    ax.relim()
    ax.autoscale_view()
    return line,

def read_and_plot():
    global x_data, y_data
    while ser.in_waiting:
        try:
            # Read and decode the data from serial
            line = ser.readline().decode('utf-8').strip()
            if line:
                # Parse the timestamp and thrust
                timestamp, thrust = map(float, line.split(','))
                
                # Write the data to the CSV file
                with open(csv_filename, 'a', newline='') as csvfile:
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerow([timestamp, thrust])
                
                # Update the data for the plot
                x_data.append(timestamp/1000000)
                y_data.append(thrust)
        except Exception as e:
            print(f"Error reading data: {e}")

ani = animation.FuncAnimation(fig, update_plot, init_func=init, interval=50, blit=True)

# Running the plot in a non-blocking way
plt.show(block=False)

try:
    while True:
        read_and_plot()
        plt.pause(0.001)  # Slight pause to allow the plot to update
except KeyboardInterrupt:
    print("Program interrupted.")
finally:
    ser.close()
    print("Serial port closed.")
