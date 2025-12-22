import paho.mqtt.client as mqtt
import json
import csv
import os
from datetime import datetime

# --- CONFIGURATION ---
# "mosquitto" is the hostname defined in your docker-compose.yml
MQTT_BROKER = "mosquitto"
MQTT_TOPIC = "health/stats"
# This path is internal to the Docker container
CSV_FILE = "/app/data/health_data.csv"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to Mosquitto internal network ✅")
        client.subscribe(MQTT_TOPIC)
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)

        # Create Timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Determine Finger Status from Boolean
        finger_detected = "Yes" if data.get("finger") is True else "No"

        # Prepare CSV Row
        # Includes SpO2 as well if available in your final sketch
        row = [
            timestamp,
            data.get("temp", "N/A"),
            data.get("alcohol", "N/A"),
            data.get("bpm", "N/A"),
            data.get("spo2", "N/A"),
            data.get("status", "Unknown"),
            finger_detected
        ]

        # Ensure directory exists and write to CSV
        os.makedirs(os.path.dirname(CSV_FILE), exist_ok=True)
        file_exists = os.path.isfile(CSV_FILE)

        with open(CSV_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            # Write header if new file
            if not file_exists or os.stat(CSV_FILE).st_size == 0:
                writer.writerow(["Timestamp", "Temp(C)", "Alcohol", "BPM", "SpO2", "Air Status", "Finger"])
            writer.writerow(row)

        print(f"Data Logged: {timestamp} | BPM: {data.get('bpm')} | Finger: {finger_detected}")

    except Exception as e:
        print(f"Logging Error: {e}")

# Setup MQTT Client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker service
client.connect(MQTT_BROKER, 1883, 60)

# Block and listen
client.loop_forever()
