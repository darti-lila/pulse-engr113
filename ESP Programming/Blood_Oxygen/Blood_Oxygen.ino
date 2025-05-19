#include <Wire.h>
#include "MAX30105.h" // SparkFun MAX3010x library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;

// Variables for calculation
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100;  // Calculate SpO2 by this sampling interval
float ESpO2 = 0;
double FSpO2 = 0.7;  // Filter factor for estimated SpO2
double frate = 0.95; // Low pass filter for IR/red LED value
#define TIMETOBOOT 3000
#define SCALE      88.0
#define SAMPLING   100
#define FINGER_ON  30000
#define USEFIFO

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Running...");

  // OLED initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("PulseOx");
  display.display();
  delay(2000);
  display.clearDisplay();

  // Initialize MAX30102 sensor
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring.");
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("MAX30102 not found");
    display.display();
    delay(2000);
  }

  byte ledBrightness = 0x7F;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 200;
  int pulseWidth = 411;
  int adcRange = 16384;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY();
}

void loop() {
  uint32_t ir, red;
  double fred, fir;
  double SpO2 = 0;

#ifdef USEFIFO
  particleSensor.check(); // Read up to 3 samples

  while (particleSensor.available()) {
    red = particleSensor.getFIFORed();
    ir  = particleSensor.getFIFOIR();
#endif

    i++;
    fred = (double)red;
    fir  = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);
    aveir = aveir * frate + (double)ir * (1.0 - frate);
    sumredrms += (fred - avered) * (fred - avered);
    sumirrms += (fir - aveir) * (fir - aveir);

    if ((i % SAMPLING) == 0) {
      if ( millis() > TIMETOBOOT) {
        // Serial output
        Serial.print("Red: "); Serial.print(red); Serial.print(", ");
        Serial.print("Infrared: "); Serial.print(ir); Serial.print(".    ");

        // OLED output
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0,0);

        if (ir < FINGER_ON) {
          Serial.println("No finger detected");
          display.setTextSize(2);
          display.setCursor(0,0);
          display.println("No finger");
          display.setCursor(0,30);
          display.println("detected");
        } else {
          Serial.print("Oxygen % = ");
          Serial.print(ESpO2);
          Serial.println("%");

          display.setTextSize(2);
          display.setCursor(0,0);
          display.print("SpO2:");
          display.print(ESpO2,1);
          display.println("%");
          display.setTextSize(1);
          display.setCursor(0, 35);
          display.print("IR: "); display.println(ir);
          display.print("Red: "); display.println(red);
        }
        display.display();
      }
    }

    if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      Serial.print("R = ");
      Serial.println(R);
      SpO2 = -45.060 * R * R + 30.354 * R + 94.845;
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;
    }
    particleSensor.nextSample();
  }
}
