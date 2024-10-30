# data_sender.py
import serial
import time
import random

# Set up serial connection to send data
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

try:
    while True:
        # Generate random values for x, y, z, and battery
        x = random.uniform(-100, 100)
        y = random.uniform(-100, 100)
        z = random.uniform(-100, 100)
        battery = random.randint(0, 9)

        # Format data as "x,y,z,battery"
        data_x= f"x: {x:.2f}\n"
        data_y= f"y: {y:.2f}\n"
        data_z= f"z: {z:.2f}\n"
        data_battery= f"Voltaje: {battery}\n"
        
        # Send data to the serial port
        ser.write(data_x.encode('utf-8'))
        time.sleep(0.1)
        ser.write(data_y.encode('utf-8'))
        time.sleep(0.1)
        ser.write(data_z.encode('utf-8'))
        time.sleep(0.1)
        ser.write(data_battery.encode('utf-8'))
        time.sleep(0.1)

        # Wait a moment before sending the next value

except KeyboardInterrupt:
    print("Data sending interrupted by user")

finally:
    ser.close()
    print("Serial connection closed")
