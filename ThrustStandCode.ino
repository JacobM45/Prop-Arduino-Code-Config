#include <Wire.h>
#include <Servo.h>
#include "HX711.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Test settings
const float calibrationFactor = 1.0;   // Set after calibrating the load cell
const int iterations = 25;
const unsigned long collectionTime = 5000;
const float bottomRange = 0.0;
const float topRange = 20.0;           // Start at 20%, not 100%
const unsigned long stabilizationTime = 1000;

// Raspberry Pi Pico pins
const int sensorIn = 28;               // GP28 / ADC2
const float adcReferenceVoltage = 3.3;
const int adcResolutionBits = 12;

// Current sensor sensitivity.
// 32.0 is appropriate only if your sensor is a 32 mV/A model.
const float mVperAmp = 32.0;

// Emergency-stop input: connect a normally-open button from GP3 to GND.
const int eStopPin = 3;

// HX711 pins — CHANGE THESE to your actual wiring.
const int HX711_DOUT_PIN = 6;
const int HX711_SCK_PIN = 7;

// TCRT5000 RPM sensor settings
const int rpmSensorPin = 2;
const int pulsesPerRevolution = 1;
volatile unsigned long rpmPulseCount = 0;

void rpmInterrupt() {
  rpmPulseCount++;
}

float calculateRPM(unsigned long pulseCount, unsigned long duration) {
  if (duration == 0 || pulsesPerRevolution <= 0) {
    return 0.0;
  }
  return (pulseCount * 60000.0) / (duration * pulsesPerRevolution);
}

bool emergencyStopActive() {
  return digitalRead(eStopPin) == LOW;
}

void stopMotor() {
  ESC.writeMicroseconds(1000);
  Serial.println("MOTOR STOPPED");
}

HX711 scale;
Servo ESC;
Adafruit_ADXL345_Unified accelerometer = Adafruit_ADXL345_Unified(12345);

float readLoadCellAverage(HX711 &scale, unsigned long duration);
bool readAccelerometerAverage(unsigned long duration, float &averageX, float &averageY, float &averageZ);
void thrustTest(int iterations = 10, unsigned long collectionTime = 5000, float bottomRange = 0, float topRange = 20);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Wire.begin();   // default Pico I2C pins: SDA=GP0, SCL=GP1

  if (!accelerometer.begin()) {
    Serial.println("ADXL345 not detected!");
    while (1) delay(100);
  }

  accelerometer.setRange(ADXL345_RANGE_16_G);
  accelerometer.setDataRate(ADXL345_DATARATE_100_HZ);

  analogReadResolution(adcResolutionBits);

  pinMode(eStopPin, INPUT_PULLUP);

  scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  scale.set_scale(calibrationFactor);

  Serial.println("Taring load cell. Keep the stand unloaded.");
  delay(1000);
  scale.tare();
  Serial.println("Load cell tare complete.");

  pinMode(rpmSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmSensorPin), rpmInterrupt, FALLING);

  ESC.attach(9, 1000, 2000);
  stopMotor();

  Serial.println();
  Serial.println("SYSTEM READY");
  Serial.println("Remove the propeller for initial testing.");
  Serial.println("Send START followed by Enter to begin.");
  Serial.println("Send STOP to stop the test.");

  while (true) {
    if (emergencyStopActive()) {
      stopMotor();
      delay(100);
      continue;
    }

    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      command.toUpperCase();

      if (command == "START") {
        Serial.println("Starting test in 5 seconds.");
        Serial.println("Confirm that the test stand is secure.");
        delay(5000);

        if (!emergencyStopActive()) {
          thrustTest(
            iterations,
            collectionTime,
            bottomRange,
            min(topRange, 20.0f)
          );
        } else {
          stopMotor();
        }

        Serial.println("Test finished. Motor is stopped.");
      } else if (command == "STOP") {
        stopMotor();
      } else {
        Serial.println("Unknown command. Type START or STOP.");
      }
    }

    delay(10);
  }
}

void loop() {
  // everything runs in setup()
}

bool readAccelerometerAverage(
  unsigned long duration,
  float &averageX,
  float &averageY,
  float &averageZ) {

  float sumX = 0.0;
  float sumY = 0.0;
  float sumZ = 0.0;
  unsigned long sampleCount = 0;

  unsigned long startTime = millis();

  while (millis() - startTime < duration) {
    if (emergencyStopActive()) {
      stopMotor();
      return false;
    }

    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      command.toUpperCase();

      if (command == "STOP") {
        stopMotor();
        return false;
      }
    }

    sensors_event_t event;
    accelerometer.getEvent(&event);

    sumX += event.acceleration.x;
    sumY += event.acceleration.y;
    sumZ += event.acceleration.z;
    sampleCount++;

    delay(2);
  }

  if (sampleCount == 0) {
    averageX = 0.0;
    averageY = 0.0;
    averageZ = 0.0;
    return false;
  }

  averageX = sumX / sampleCount;
  averageY = sumY / sampleCount;
  averageZ = sumZ / sampleCount;
  return true;
}

void thrustTest(
  int iterations,
  unsigned long collectionTime,
  float bottomRange,
  float topRange) {

  if (iterations < 2) {
    Serial.println("ERROR: iterations must be at least 2.");
    stopMotor();
    return;
  }

  topRange = constrain(topRange, bottomRange, 20.0f);

  float increment = (topRange - bottomRange) / (iterations - 1);

  Serial.println("Data collection starts...");
  Serial.println("throttle,thrust,rpm,accelX_g,accelY_g,accelZ_g,amps");

  for (int i = 0; i < iterations; i++) {

    if (emergencyStopActive()) {
      stopMotor();
      return;
    }

    float throttle = constrain(
      bottomRange + i * increment,
      0.0f,
      20.0f
    );

    int pulseWidth = 1000 + (int)((throttle / 100.0f) * 1000.0f);
    pulseWidth = constrain(pulseWidth, 1000, 1200);
    ESC.writeMicroseconds(pulseWidth);

    delay(stabilizationTime);

    // Reset and start RPM counting
    rpmPulseCount = 0;
    unsigned long rpmStart = millis();

    // Collect accelerometer and RPM data during same time period
    float accelerationX;
    float accelerationY;
    float accelerationZ;

    bool accelOK = readAccelerometerAverage(
      collectionTime,
      accelerationX,
      accelerationY,
      accelerationZ
    );

    // Read the load cell after the acceleration collection.
    float thrust = readLoadCellAverage(scale, collectionTime);

    noInterrupts();
    unsigned long countedPulses = rpmPulseCount;
    interrupts();

    unsigned long rpmDuration = millis() - rpmStart;
    float rpm = calculateRPM(countedPulses, rpmDuration);

    // Read current sensor
    int rawADC = analogRead(sensorIn);

    const float adcMaximum =
      (1UL << adcResolutionBits) - 1;

    float sensorVoltage =
      (rawADC / adcMaximum) * adcReferenceVoltage;

    // This assumes the current sensor's zero-current output is half
    // of its 3.3 V supply. Measure and calibrate this value in practice.
    float zeroCurrentOffset = adcReferenceVoltage / 2.0;
    float netVoltage = sensorVoltage - zeroCurrentOffset;
    float amps = (netVoltage * 1000.0) / mVperAmp;

    // Print CSV data
    if (accelOK) {
      float axG = accelerationX / 9.80665;
      float ayG = accelerationY / 9.80665;
      float azG = accelerationZ / 9.80665;

      Serial.print(throttle, 2);
      Serial.print(",");
      Serial.print(thrust, 4);
      Serial.print(",");
      Serial.print(rpm, 1);
      Serial.print(",");
      Serial.print(axG, 4);
      Serial.print(",");
      Serial.print(ayG, 4);
      Serial.print(",");
      Serial.print(azG, 4);
      Serial.print(",");
      Serial.println(amps, 3);
    } else {
      Serial.print(throttle, 2);
      Serial.print(",");
      Serial.print(thrust, 4);
      Serial.print(",");
      Serial.print(rpm, 1);
      Serial.println(",ERROR,ERROR,ERROR,0");
    }
  }

  ESC.writeMicroseconds(1000);
  Serial.println("END OF DATA COLLECTION");
}

float readLoadCellAverage(HX711 &scale, unsigned long duration) {

  float dataSum = 0.0;
  unsigned long dataCount = 0;

  unsigned long startTime = millis();

  while (millis() - startTime < duration) {
    if (emergencyStopActive()) {
      stopMotor();
      return 0.0;
    }

    if (scale.is_ready()) {
      float weightReading = scale.get_units(1);
      dataSum += weightReading;
      dataCount++;
    }
  }

  if (dataCount == 0) {
    return 0.0;
  }

  return dataSum / dataCount;
}
