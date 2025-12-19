#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- 1. CONFIGURATION (EDIT THIS) ---
const char *ssid = "Xiaomi 15";            // <--- WIFI NAME
const char *password = "12345678999";      // <--- WIFI PASSWORD
const char *mqtt_server = "10.109.83.181"; // <--- PC IP ADDRESS

// --- 2. PIN DEFINITIONS ---
#define MQ3_PIN 36     // MQ-3 Analog (Voltage Divider) -> GPIO 36
#define ONE_WIRE_BUS 4 // DS18B20 Data -> GPIO 4
// MAX30102 uses default I2C: SDA=21, SCL=22

// --- 3. OBJECTS ---
WiFiClient espClient;
PubSubClient client(espClient);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MAX30105 particleSensor;

// MQ-3
int mq3_baseline = 4095;

// MAX30102
const byte RATE_SIZE = 6; // Increased from 4 to 6 for smoother averages
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg = 0;

// Timers
long lastMsg = 0;

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  // Non-blocking reconnect would be better, but for simplicity we block briefly
  if (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32_Health_Hub"))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" (will try again next loop)");
    }
  }
}

void setup()
{
  // Disable Brownout Detector to prevent WiFi/MQ-3 crashes
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);

  // Power stabilization delay
  Serial.println("System starting... wait 2s for power to stabilize.");
  delay(2000);

  // 1. Setup MQ-3
  analogReadResolution(12);

  // 2. Setup DS18B20
  sensors.begin();
  // CRITICAL FIX: Don't wait 750ms for temp. Return immediately.
  sensors.setWaitForConversion(false);
  // Start the first conversion now, so data is ready for the first loop
  sensors.requestTemperatures();

  // 3. Setup MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30102 not found. Check wiring/power.");
    // We continue anyway so the other sensors still work
  }
  else
  {
    Serial.println("MAX30102 Found.");
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A); // Low Red LED current
    particleSensor.setPulseAmplitudeGreen(0);
  }

  // 4. Setup Network
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop()
{
  // --- CRITICAL FAST LOOP: HEART RATE ---
  // This must run fast (every loop) to detect beats.
  // With Async Temp enabled, this loop is no longer blocked!
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  // --- SLOW LOOP: MQTT & OTHER SENSORS ---
  // Run this every 2 seconds
  long now = millis();
  if (now - lastMsg > 2000)
  {
    lastMsg = now;

    // Check MQTT Connection
    if (!client.connected())
    {
      reconnect();
    }
    client.loop();

    // 1. Read Temp (The conversion happened during the last 2 seconds)
    // We don't call requestTemperatures() here yet. We just read the result.
    float rawTemp = sensors.getTempCByIndex(0);

    // Start the NEXT conversion (It will happen in the background)
    sensors.requestTemperatures();

    float displayTemp = rawTemp;

    // "Human Core" estimation
    if (rawTemp > 28.0)
      displayTemp = rawTemp + 5.5;
    if (rawTemp == -127.00)
      displayTemp = 0.0; // Error handling

    // 2. Read Alcohol
    int alcValue = analogRead(MQ3_PIN);
    // Dynamic Baseline (Auto-Calibration)
    if (alcValue < mq3_baseline)
      mq3_baseline = alcValue;

    // 3. Determine Alcohol Status
    String alcStatus = "Clean";
    if (alcValue >= (mq3_baseline + 400) && alcValue < (mq3_baseline + 2000))
      alcStatus = "Detected";
    else if (alcValue >= (mq3_baseline + 2000))
      alcStatus = "High";

    // 4. Determine Finger Status
    String fingerStatus = (irValue > 50000) ? "true" : "false";

    // 5. Construct JSON Payload
    String payload = "{";
    payload += "\"temp\":";
    payload += String(displayTemp);
    payload += ",";
    payload += "\"alcohol\":";
    payload += String(alcValue);
    payload += ",";
    payload += "\"status\":\"";
    payload += alcStatus;
    payload += "\",";
    payload += "\"bpm\":";
    payload += String(beatAvg);
    payload += ",";
    payload += "\"finger\":";
    payload += fingerStatus;
    payload += "}";

    // 6. Publish to MQTT
    client.publish("health/stats", payload.c_str());

    // 7. Print to Serial (Plotter Friendly)
    Serial.print("Temp:");
    Serial.print(displayTemp);
    Serial.print(",");
    Serial.print("Alc:");
    Serial.print(alcValue);
    Serial.print(",");
    Serial.print("BPM:");
    Serial.print(beatAvg);
    Serial.print(",");
    Serial.print("IR:");
    Serial.println(irValue);
  }
}