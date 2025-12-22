#include "MAX30105.h"
#include "heartRate.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

// wifi config
const char *ssid = "Xiaomi 15";            // WIFI NAME
const char *password = "12345678999";      // WIFI PASSWORD
const char *mqtt_server = "10.109.83.181"; // PC IP ADDRESS

// defines
#define MQ3_PIN 36                 // MQ-3 Analog (Voltage Divider) -> GPIO 36
#define ONE_WIRE_BUS 4             // DS18B20 Data -> GPIO 4
#define MESSAGE_DELAY_TIME_MS 2000 // 2 seconds
#define FINGER_DETECT_THRESHOLD 50000 // IR value threshold for finger detection
#define SPO2_SAMPLE_SIZE 100          // Number of samples for SpO2 calculation

// objects
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
OneWire onewire(ONE_WIRE_BUS);
DallasTemperature sensors(&onewire);
MAX30105 particle_sensor;

// MQ-3
int mq3_baseline = 4095;

// MAX30102 heart rate
const byte RATE_SIZE = 6;
byte rates[RATE_SIZE];
byte rate_spot = 0;
long last_beat = 0;
float bpm_current;
int bpm_average = 0;

// MAX30102 SpO2 variables
double average_red = 0;
double average_ir = 0;
double sum_red_rms = 0;
double sum_ir_rms = 0;
double spo2 = 0;
double e_spo2 = 95.0; // Initial estimated SpO2
int spo2_counter = 0;

// Timers
long last_msg_time = 0;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect_wifi() {
  if (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqtt_client.connect("ESP32_Health_Hub")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" (will try again next loop)");
    }
  }
}

void setup() {
  // Disable Brownout Detector to prevent WiFi/MQ-3 crashes
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);

  // Power stabilization delay
  Serial.println("System starting... wait 2s...");
  delay(2000);

  // 1. Setup MQ-3
  analogReadResolution(12);

  // 2. Setup DS18B20
  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  // 3. Setup MAX30102
  if (!particle_sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring/power.");
  } else {
    Serial.println("MAX30102 found.");
    particle_sensor.setup();
    particle_sensor.setPulseAmplitudeRed(0x0A); // Low Red LED current
    particle_sensor.setPulseAmplitudeGreen(0);
  }

  // 4. Setup Network
  setup_wifi();
  mqtt_client.setServer(mqtt_server, 1883);
}

void loop() {
  // Fast loop for heart rate & SpO2
  long ir_value = particle_sensor.getIR();
  long red_value = particle_sensor.getRed();

  // 1. Heart Rate Logic
  if (checkForBeat(ir_value) == true) {
    long delta = millis() - last_beat;
    last_beat = millis();

    bpm_current = 60 / (delta / 1000.0);

    if (bpm_current < 255 && bpm_current > 20) {
      rates[rate_spot++] = (byte)bpm_current;
      rate_spot %= RATE_SIZE;

      bpm_average = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        bpm_average += rates[x];
      bpm_average /= RATE_SIZE;
    }
  }

  // 2. SpO2 Logic (Continuous RMS method)
  if (ir_value > FINGER_DETECT_THRESHOLD) {
    // Remove DC component (simple IIR filter)
    average_red = 0.95 * average_red + 0.05 * red_value;
    average_ir = 0.95 * average_ir + 0.05 * ir_value;

    double ac_red = red_value - average_red;
    double ac_ir = ir_value - average_ir;

    // Accumulate squared AC (for RMS)
    sum_red_rms += ac_red * ac_red;
    sum_ir_rms += ac_ir * ac_ir;
    spo2_counter++;

    // Calculate SpO2 every N samples
    if (spo2_counter >= SPO2_SAMPLE_SIZE) {
      double red_rms = sqrt(sum_red_rms / SPO2_SAMPLE_SIZE);
      double ir_rms = sqrt(sum_ir_rms / SPO2_SAMPLE_SIZE);

      // Ratio of Ratios
      double r = (red_rms / average_red) / (ir_rms / average_ir);

      // Empirical formula for SpO2
      spo2 = 110.0 - 18.0 * r;

      // Limit to realistic human range (80-100)
      if (spo2 > 100) spo2 = 100;
      if (spo2 < 80) spo2 = 80;

      // Smooth the result
      e_spo2 = 0.9 * e_spo2 + 0.1 * spo2;

      // Reset counters
      sum_red_rms = 0;
      sum_ir_rms = 0;
      spo2_counter = 0;
    }
  } else {
    // Reset averages if finger removed
    e_spo2 = 0.0;
    bpm_average = 0;
  }

  // Slow loop for other sensors and MQTT
  long now = millis();
  if (now - last_msg_time > MESSAGE_DELAY_TIME_MS) {
    last_msg_time = now;

    // Check MQTT Connection
    if (!mqtt_client.connected()) {
      reconnect_wifi();
    }
    mqtt_client.loop();

    float temp_raw = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();

    float temp_display = temp_raw;

    // "Human Core" estimation
    if (temp_raw > 28.0)
      temp_display = temp_raw + 5.5;
    if (temp_raw == -127.00)
      temp_display = 0.0; // Error handling

    // 2. Read Alcohol
    int alcohol_value = analogRead(MQ3_PIN);
    // Dynamic Baseline (Auto-Calibration)
    if (alcohol_value < mq3_baseline)
      mq3_baseline = alcohol_value;

    // 3. Determine Alcohol Status
    String alcohol_status = "Clean";
    if (alcohol_value >= (mq3_baseline + 400) && alcohol_value < (mq3_baseline + 2000))
      alcohol_status = "Detected";
    else if (alcohol_value >= (mq3_baseline + 2000))
      alcohol_status = "High";

    String finger_status = (ir_value > FINGER_DETECT_THRESHOLD) ? "true" : "false";

    String payload = "{";
    payload += "\"temp\":";
    payload += String(temp_display);
    payload += ",";
    payload += "\"alcohol\":";
    payload += String(alcohol_value);
    payload += ",";
    payload += "\"status\":\"";
    payload += alcohol_status;
    payload += "\",";
    payload += "\"bpm\":";
    payload += String(bpm_average);
    payload += ",";
    payload += "\"spo2\":";
    payload += String(e_spo2);
    payload += ",";
    payload += "\"finger\":";
    payload += finger_status;
    payload += "}";

    mqtt_client.publish("health/stats", payload.c_str());

    // print to serial (CSV format without labels)
    Serial.print(temp_display);
    Serial.print(",");
    Serial.print(alcohol_value);
    Serial.print(",");
    Serial.print(bpm_average);
    Serial.print(",");
    Serial.print(e_spo2);
    Serial.print(",");
    Serial.println(ir_value);
  }
}
