from json import dumps
from random import randint
from time import sleep
import paho.mqtt.client as mqtt
from serial import Serial

# MQTT broker details
broker = "iot.eie.ucr.ac.cr"
port = 1883
topic = "v1/devices/me/telemetry"
access_token = "YI3VpgHK1lc5iCwO2JvR"

# Callback for connection
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to ThingsBoard")
    else:
        print("Connection failed")

# Set up serial connection
ser = Serial('/dev/ttyACM1', 115200, timeout=1)
sleep(2)

# Set up the client
client = mqtt.Client()
client.username_pw_set(access_token)
client.on_connect = on_connect
client.connect(broker, port, keepalive=60)

# Start the loop to handle callbacks (blocking mode)
client.loop_start()

try:
    while True:
        # Read line
        data = ser.readline().decode('utf-8').strip()
        if data:
            print(data)
            data_to_send = data.split(': ')
            label = data_to_send[0]
            value = data_to_send[1]

            # Publish data to MQTT broker
            client.publish(topic, dumps({label: value}), qos=1)

        #sleep(0.1)  # Allow time for publishing

except KeyboardInterrupt:
    print("Exiting program")

finally:
    client.loop_stop()  # Stop the loop when exiting
    ser.close()