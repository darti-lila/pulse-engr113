#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define custom I2C buses
TwoWire I2C_MAX30102 = TwoWire(0); // First I2C bus for MAX30102
TwoWire I2C_OLED = TwoWire(1);     // Second I2C bus for OLED

#define MAX30102_SDA 19
#define MAX30102_SCL 18
#define OLED_SDA     21
#define OLED_SCL     22

MAX30105 particleSensor;

// OLED display setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);

const long FINGER_THRESHOLD = 50000;
#define HR_AVG_SIZE 8
float hrBuffer[HR_AVG_SIZE];
int hrIndex = 0;
int hrCount = 0;
float avgHR = 0; // Store the latest average HR

void setup() {
  Serial.begin(115200);

  // Initialize I2C buses
  I2C_MAX30102.begin(MAX30102_SDA, MAX30102_SCL);
  I2C_OLED.begin(OLED_SDA, OLED_SCL);

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while(1);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  // Initialize MAX30102 on its own I2C bus
  if (!particleSensor.begin(I2C_MAX30102, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }
  particleSensor.setup(); // Use default settings
  particleSensor.setPulseAmplitudeRed(0x0A);   // Low red LED
  particleSensor.setPulseAmplitudeGreen(0);    // Turn off green LED
}

void loop() {
  long irValue = particleSensor.getIR();

  display.clearDisplay();
  display.setCursor(0, 0);

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
      avgHR = 0;
      for (int i = 0; i < hrCount; i++) {
        avgHR += hrBuffer[i];
      }
      avgHR /= hrCount;

      Serial.print("Heart Rate (avg): ");
      Serial.print(avgHR, 1); // 1 decimal place
      Serial.println(" BPM");
    }

    // Always display the last calculated avgHR if finger is present
    display.setTextSize(2);
    display.print("HR: ");
    if (hrCount > 0) {
      display.print(avgHR, 1);
      display.println(" BPM");
    } else {
      display.println("-- BPM");
    }
  } else {
    // No finger detected
    Serial.println("No finger detected");
    hrCount = 0; // Reset buffer when finger is removed
    hrIndex = 0;
    avgHR = 0;   // Reset avgHR

    display.setTextSize(2);
    display.println("No finger");
    display.println("detected");
    delay(500); // Slow down message when finger is not present
  }

  display.display();
  delay(20); // Sensor polling interval
}
