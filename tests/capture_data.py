import serial
import time

# Set up serial connection to receive data
ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

while True:
    # Read a line of data from the serial port
    data = ser.readline().decode('utf-8').strip()

    # Check if data is not empty
    if data:
        print(f"Received: {data}")
        
        # Here you would send `data` to the MQTT broker
        # For example: mqtt_client.publish("your_topic", data)

