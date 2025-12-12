#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// defines
#define MQ3_PIN 36
#define ONE_WIRE_BUS 4
#define MESSAGE_TIME 3000

// objects
WiFiClient wifi_client;
PubSubClient client(wifi_client);
OneWire one_wire(ONE_WIRE_BUS);
DallasTemperature sensors(&one_wire);

// variables
int mq3_baseline = 4095;
long last_message_time = 0;

const char *wifi_name = "Xiaomi 15";
const char *wifi_pass = "12345678999";
const char *mqtt_server = "10.109.83.181";

void wifi_setup()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_name);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_name, wifi_pass);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqtt_reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32HealthTracker"))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
  sensors.begin();

  wifi_setup();
  client.setServer(mqtt_server, 1883);
}

void loop()
{
  if (!client.connected())
  {
    mqtt_reconnect();
  }
  client.loop();

  long now = millis();
  if (now - last_message_time > MESSAGE_TIME)
  {
    last_message_time = now;

    // read details
    sensors.requestTemperatures();
    float temp_raw = sensors.getTempCByIndex(0);
    int mq3_value = analogRead(MQ3_PIN);

    if (mq3_value < mq3_baseline)
      mq3_baseline = mq3_value;

    float temp_value = temp_raw;
    if (temp_raw < 20.0)
      temp_value += 5.5;

    String payload = "{";

    // add temp
    payload += "\"temp\":";
    if (temp_raw == -127)
      payload += "null";
    else
      payload += String(temp_value);
    payload += ",";

    // add alcohol
    payload += "\"alcohol\":";
    payload += String(mq3_value);
    payload += ",";

    // add body status
    payload += "\"status\":\"";
    if (mq3_value < (mq3_baseline + 400))
      payload += "Clean";
    else if (mq3_value < (mq3_baseline + 2000))
      payload += "Light";
    else
      payload += "Heavy";
    payload += "\"}";

    // send to mqtt broker
    Serial.print("Payload: ");
    Serial.println(payload);
    client.publish("health/stats", payload.c_str());
  }
}