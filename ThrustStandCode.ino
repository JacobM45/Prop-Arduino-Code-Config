#include <Wire.h>
#include <Servo.h>
#include "HX711.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Editable variables
const float calibrationFactor = 1.0;
const int iterations = 25;
const unsigned long collectionTime = 5000;
const float bottomRange = 0;
const float topRange = 100;
const unsigned long stabilizationTime = 1000;

const int sensorIn = 28;
const float mVperAmp = 32.0;
const float vccVoltage = 5.0;

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

HX711 scale;
Servo ESC;
Adafruit_ADXL345_Unified accelerometer = Adafruit_ADXL345_Unified(12345);

float readLoadCellAverage(HX711 &scale, unsigned long duration);
bool readAccelerometerAverage(unsigned long duration, float &averageX, float &averageY, float &averageZ);
void thrustTest(int iterations = 10, unsigned long collectionTime = 5000, float bottomRange = 0, float topRange = 100);

void setup() {
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  if (!accelerometer.begin()) {
    Serial.println("ADXL345 not detected!");
    while (1) delay(100);
  }

  accelerometer.setRange(ADXL345_RANGE_16_G);
  accelerometer.setDataRate(ADXL345_DATARATE_100_HZ);

  pinMode(rpmSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmSensorPin), rpmInterrupt, FALLING);

  ESC.attach(9, 1000, 2000);
  ESC.writeMicroseconds(1000);

  Serial.println("RPM sensor enabled.");
  Serial.println("Testing tips:");
  Serial.println("1. Start with pulsesPerRevolution = 1.");
  Serial.println("2. If sensor only sees blade passes, use the blade count.");
  Serial.println("3. Try FALLING first, then RISING if no pulses register.");
  Serial.println("4. Keep the sensor close to the reflector/prop path.");

  delay(2000);

  thrustTest(iterations, collectionTime, bottomRange, topRange);
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

  float increment = (topRange - bottomRange) / (iterations - 1);

  Serial.println("Data collection starts...");
  Serial.println("throttle,thrust,rpm,accelX_g,accelY_g,accelZ_g,amps");

  for (int i = 0; i < iterations; i++) {

    float throttle = bottomRange + i * increment;

    int pulseWidth = 1000 + (throttle / 100.0) * 1000;
    ESC.writeMicroseconds(pulseWidth);

    delay(stabilizationTime);

    // Reset and start RPM counting
    rpmPulseCount = 0;
    unsigned long rpmStart = millis();

    // Collect thrust data
    float thrust = 0;

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

    unsigned long rpmDuration = millis() - rpmStart;
    float rpm = calculateRPM(rpmPulseCount, rpmDuration);

    // Read current sensor
    int rawADC = analogRead(sensorIn);
    float sensorVoltage = (rawADC / 1023.0) * vccVoltage;
    float zeroCurrentOffset = vccVoltage / 2.0;
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
