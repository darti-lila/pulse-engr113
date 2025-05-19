#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAX30105 sensor
MAX30105 particleSensor;
const long FINGER_THRESHOLD = 30000; // Use lower threshold for SpO2 logic

// SpO2 calculation variables
double avered = 0, aveir = 0, sumirrms = 0, sumredrms = 0;
int i = 0;
const int Num = 100;  // SpO2 calculation interval
float ESpO2 = 0;
double FSpO2 = 0.7;  // Filter factor for estimated SpO2
double frate = 0.95; // Low pass filter for IR/red LED value
#define TIMETOBOOT 3000
#define SAMPLING   100

// Heart rate moving average
#define HR_AVG_SIZE 8
float hrBuffer[HR_AVG_SIZE];
int hrIndex = 0;
int hrCount = 0;
float lastAvgHR = 0;
bool validHR = false;
bool fingerDetected = false;

void setup() {
  Serial.begin(115200);
  Serial.println("PulseOx + Heart Rate Starting...");

  // OLED initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("PulseOx+HR");
  display.display();
  delay(2000);

  // Initialize MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found. Check wiring.");
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Sensor Error");
    display.display();
    while (1);
  }
  // Sensor setup: Red & IR LEDs, 200Hz, 411us pulse, 16-bit ADC
  byte ledBrightness = 0x7F;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 200;
  int pulseWidth = 411;
  int adcRange = 16384;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeGreen(0); // Turn off green LED
  particleSensor.enableDIETEMPRDY();
}

void loop() {
  static long lastBeat = 0;
  static bool firstBeat = true;
  static unsigned long lastDisplayUpdate = 0;
  const unsigned long displayInterval = 150; // ms

  // ----------- Data Acquisition -----------
  uint32_t ir = particleSensor.getIR();
  uint32_t red = particleSensor.getRed();
  fingerDetected = ir > FINGER_THRESHOLD;
  validHR = false;

  // Heart Rate Calculation
  if (fingerDetected) {
    if (checkForBeat(ir)) {
      long now = millis();
      if (firstBeat) {
        // Ignore the first beat after finger placement
        lastBeat = now;
        firstBeat = false;
      } else {
        long delta = now - lastBeat;
        lastBeat = now;
        if (delta > 300 && delta < 2000) { // Accept only realistic intervals (30-200 BPM)
          float heartRate = 60.0 / (delta / 1000.0);
          // Store in moving average buffer
          hrBuffer[hrIndex] = heartRate;
          hrIndex = (hrIndex + 1) % HR_AVG_SIZE;
          if (hrCount < HR_AVG_SIZE) hrCount++;
          validHR = true;
        }
      }
    }
  } else {
    // Reset HR buffer and state when finger is removed
    hrCount = 0;
    hrIndex = 0;
    firstBeat = true;
    lastBeat = 0;
  }

  // Heart Rate Moving Average
  if (fingerDetected && hrCount > 0) {
    float avgHR = 0;
    for (int j = 0; j < hrCount; j++) avgHR += hrBuffer[j];
    avgHR /= hrCount;
    lastAvgHR = avgHR;
    validHR = true;
  } else {
    validHR = false;
  }

  // ----------- SpO2 Calculation -----------
  double fred = (double)red;
  double fir  = (double)ir;
  avered = avered * frate + (double)red * (1.0 - frate);
  aveir = aveir * frate + (double)ir * (1.0 - frate);
  sumredrms += (fred - avered) * (fred - avered);
  sumirrms += (fir - aveir) * (fir - aveir);
  i++;

  if ((i % Num) == 0 && fingerDetected && millis() > TIMETOBOOT) {
    double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
    double SpO2 = -45.060 * R * R + 30.354 * R + 94.845;
    ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
    sumredrms = 0.0; sumirrms = 0.0; i = 0;
  }

  // ----------- Display Phase -----------
  if (millis() - lastDisplayUpdate > displayInterval) {
    display.clearDisplay();
    display.setCursor(0, 0);

    if (!fingerDetected) {
      display.setTextSize(2);
      display.println("No finger");
      Serial.println("No finger detected");
    } else if (fingerDetected && !validHR) {
      display.setTextSize(2);
      display.println("Detecting...");
      Serial.println("Detecting...");
    } else {
      display.setTextSize(2);
      display.print("HR: ");
      display.print(lastAvgHR, 1);
      display.println(" BPM");
      display.setTextSize(2);
      display.print("SpO2: ");
      display.print(ESpO2, 1);
      display.println("%");
      Serial.print("Heart Rate (avg): ");
      Serial.print(lastAvgHR, 1);
      Serial.print(" BPM, SpO2: ");
      Serial.print(ESpO2, 1);
      Serial.println("%");
    }
    display.display();
    lastDisplayUpdate = millis();
  }

  delay(20); // Sensor polling interval
}
