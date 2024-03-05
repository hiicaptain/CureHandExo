import serial
import csv
from datetime import datetime

# Open serial port
PORT = '/dev/ttyACM0'
SPEED = 115200
ser = serial.Serial(PORT, SPEED, timeout=0.1)

# Generate a timestamped filename
filename = 'sensor_data_{}.csv'.format(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

# Open (or create) CSV file
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Sensor 1", "Sensor 2", "Sensor 3", "Sensor 4", "Timestamp"]) # Write header

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()  # Read a line of data
            data = line.split(",")  # Split the line into individual data points
            try:
                # Attempt to divide each sensor value by 100
                data = [str(float(value)/100) for value in data]
                writer.writerow(data)  # Write data to CSV
            except ValueError:
                # Attempt to divide each sensor value by 100
                data = [0,0,0,0]
                data = [str(float(value)/100) for value in data]
                writer.writerow(data)  # Write data to CSV