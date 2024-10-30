from json import dumps
from random import randint
from time import sleep
import paho.mqtt.client as mqtt

# MQTT broker details
broker = "iot.eie.ucr.ac.cr"
port = 1883
topic = "v1/devices/me/telemetry"
access_token = "YI3VpgHK1lc5iCwO2JvR"
battery = 0

# Callback for connection
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to ThingsBoard")
    else:
        print("Connection failed")

# Set up the client
client = mqtt.Client()
client.username_pw_set(access_token)
client.on_connect = on_connect
client.connect(broker, port, keepalive=60)

# Start the loop to handle callbacks
client.loop_start()

#try:
while True:
    # Generate random data
    gyro_x = randint(-100, 100)
    gyro_y = randint(-100, 100)
    gyro_z = randint(-100, 100)
    battery = battery + 0.25

    # Publish each data point separately
    client.publish(topic, dumps({"x": gyro_x}), qos=1)
    print(f"Sent x: {gyro_x}")
    
    client.publish(topic, dumps({"y": gyro_y}), qos=1)
    print(f"Sent y: {gyro_y}")
    
    client.publish(topic, dumps({"z": gyro_z}), qos=1)
    print(f"Sent z: {gyro_z}")

    client.publish(topic, dumps({"battery lvl": battery}), qos=1)
    print(f"Sent battery lvl: {battery}")

    # Wait 5 seconds before sending the next set of values
    sleep(5)

# except KeyboardInterrupt:
#     print("Stopped by user")

# finally:
#     client.loop_stop()  # Stop the loop
#     client.disconnect()  # Disconnect from the broker