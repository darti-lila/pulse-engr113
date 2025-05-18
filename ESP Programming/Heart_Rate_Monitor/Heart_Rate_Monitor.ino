#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const long FINGER_THRESHOLD = 50000;

// Moving average parameters
#define HR_AVG_SIZE 8
float hrBuffer[HR_AVG_SIZE];
int hrIndex = 0;
int hrCount = 0;

void setup() {
  Serial.begin(115200);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }
  

  particleSensor.setup(); // Use default settings
  particleSensor.setPulseAmplitudeRed(0x0A);   // Low red LED
  particleSensor.setPulseAmplitudeGreen(0);    // Turn off green LED
}

void loop() {
  long irValue = particleSensor.getIR();

  if (irValue > FINGER_THRESHOLD) {
    // Finger detected, try to calculate heart rate
    if (checkForBeat(irValue)) {
      static long lastBeat = 0;
      long delta = millis() - lastBeat;
      lastBeat = millis();

      float heartRate = 60.0 / (delta / 1000.0);

      // Store heart rate in buffer
      hrBuffer[hrIndex] = heartRate;
      hrIndex = (hrIndex + 1) % HR_AVG_SIZE;
      if (hrCount < HR_AVG_SIZE) hrCount++;

      // Calculate average
      float avgHR = 0;
      for (int i = 0; i < hrCount; i++) {
        avgHR += hrBuffer[i];
      }
      avgHR /= hrCount;

      Serial.print("Heart Rate (avg): ");
      Serial.print(avgHR, 1); // 1 decimal place
      Serial.println(" BPM");
    }
  } else {
    // No finger detected
    Serial.println("No finger detected");
    hrCount = 0; // Reset buffer when finger is removed
    hrIndex = 0;
    delay(500); // Slow down message when finger is not present
  }

  delay(20); // Sensor polling interval
}
