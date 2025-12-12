#include <OneWire.h>
#include <DallasTemperature.h>

// --- CONFIGURATION ---
#define MQ3_PIN 36     // MQ-3 Analog Output -> GPIO 36 (VP)
#define ONE_WIRE_BUS 4 // DS18B20 Data Pin -> GPIO 4

// --- OBJECTS ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- VARIABLES ---
int mq3_baseline = 4095; // Dynamic baseline for alcohol (starts high)

void setup()
{
  Serial.begin(115200);

  // Setup MQ-3
  analogReadResolution(12);

  // Setup DS18B20
  sensors.begin();

  Serial.println("Sensors Initializing...");
  Serial.println("Note: If MQ-3 readings drift up, press the RST button to recalibrate.");
  delay(2000);
}

void loop()
{
  // 1. READ TEMPERATURE (DS18B20)
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // 2. READ ALCOHOL (MQ-3)
  int sensorValue = analogRead(MQ3_PIN);

  // Dynamic Baseline Logic for MQ-3
  // Only updates if value goes LOWER (finding cleaner air)
  if (sensorValue < mq3_baseline)
  {
    mq3_baseline = sensorValue;
  }

  // 3. PRINT RESULTS
  // Print Temperature
  Serial.print("Temp: ");
  if (tempC == -127.00)
  {
    Serial.print("Error");
  }
  else
  {
    Serial.print(tempC);
    Serial.print("C");
  }

  Serial.print(" | ");

  // Print Alcohol Status
  Serial.print("Alc: ");
  Serial.print(sensorValue);
  Serial.print(" (Base: ");
  Serial.print(mq3_baseline);
  Serial.print(") -> ");

  // --- UPDATED THRESHOLDS (WIDER BUFFER) ---
  // Increased buffer from 150 to 400 to ignore thermal drift
  if (sensorValue < (mq3_baseline + 400))
  {
    Serial.println("Clean Air");
  }
  else if (sensorValue >= (mq3_baseline + 400) && sensorValue < (mq3_baseline + 2000))
  {
    Serial.println("DETECTED (Light)");
  }
  else
  {
    Serial.println("HIGH CONCENTRATION!");
  }

  delay(1000);
}