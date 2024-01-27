import pymongo
import paho.mqtt.client as mqtt
from datetime import datetime

# MongoDB configuration
mongo_client = pymongo.MongoClient("mongodb://localhost:27017/")
db = mongo_client["hivetech"]
collection = db["smartbee"]
# MQTT configuration
# mqtt_broker_address = "34.133.230.216"
mqtt_topic = "movement"

import subprocess


def get_external_ip():
    # Execute the curl command to get the external IP address
    result = subprocess.run(
        ["curl", "http://ifconfig.me/"], capture_output=True, text=True
    )

    # Check if the command was successful
    if result.returncode == 0:
        # Extract and return the IP address from the output
        return result.stdout.strip()
    else:
        # Print error message if the command failed
        print("Error retrieving external IP address:", result.stderr)
        return None


def on_message(client, userdata, message):
    payload = message.payload.decode("utf-8")
    print(f"Received message: {payload}")
    # Convert MQTT timestamp to datetime
    timestamp = datetime.utcnow()  # Use current UTC time
    datetime_obj = timestamp.strftime("%Y-%m-%dT%H:%M:%S.%fZ")
    # Process the payload and insert into MongoDB with proper timestamp
    document = {"timestamp": datetime_obj, "data": payload}
    collection.insert_one(document)
    print("Data ingested into MongoDB")


# Call the function to get the external IP address
mqtt_broker_address = get_external_ip()
if mqtt_broker_address:
    print("External IP address:", mqtt_broker_address)
    client = mqtt.Client()
    client.on_message = on_message
    # Connect to MQTT broker
    client.connect(mqtt_broker_address, 1883, 60)
    # Subscribe to MQTT topic
    client.subscribe(mqtt_topic)
    # Start the MQTT loop
    client.loop_forever()
else:
    print("Failed to retrieve external IP address.")
