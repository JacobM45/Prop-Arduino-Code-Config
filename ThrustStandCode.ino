#include <Servo.h>
#include <WinsonLib.h>
#include "HX711.h"

// Editable variables
// Calibration factor = Raw Value / Known Weight
const float calibrationFactor = 1.0;  // SET CALIBRATION ONCE AND LEAVE PHYSICAL SETUP HOW IT IS
const int iterations = 25;            // HOW MANY DATA POINTS DO YOU WANT
const unsigned long collectionTime = 5000;  // DATA COLLECTION TIME FOR EACH ITERATION (milliseconds)
const float bottomRange = 0;           // WHAT RANGE OF THRUST SHOULD BE TESTED (percent)
const float topRange = 100;

const unsigned long stabilizationTime = 1000;  // TIME TO LET MOTOR DATA STABILIZE (milliseconds)

const int sensorIn = 28;      // Analog pin connected to sensor VOUT
const float mVperAmp = 32.0;  // 32 mV/A sensitivity for WCS1700 70A sensor
const float vccVoltage = 5.0;

HX711 scale;  // create the load cell object
Servo ESC;    // create servo object to control the ESC
WCS WCS1 = WCS(0, _WCS1600); // create the current sensor

// preload the functions here
struct SensorData {
    float averageThrust;
    float averageCurrent;
};

SensorData averageData(HX711 &scale, WCS WCS1, unsigned long duration);

void thrustTest(int iterations = 10,unsigned long collectionTime = 5000,float bottomRange = 0,float topRange = 100);

void setup() { // OH YEAH SETUP TIME

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  accelSetup(); //Setup the accelerometer

  ESC.attach(9, 1000, 2000);  // (pin, min pulse width, max pulse width)
  ESC.writeMicroseconds(1000);

  scale.begin(21, 20);  // pin 21 and 20
  scale.set_scale(calibrationFactor);
  scale.tare();

  delay(2000);

  thrustTest(iterations, collectionTime, bottomRange, topRange);  // Start the test!
}

void loop() {
  // everything runs in setup()
}


void thrustTest(
  int iterations,
  unsigned long collectionTime,
  float bottomRange,
  float topRange) {

  float increment = (topRange - bottomRange) / (iterations - 1);

  Serial.println("Data collection starts...");

  // Test each throttle setting
  for (int i = 0; i < iterations; i++) {

    float throttle = bottomRange + i * increment;

    // Change thrust value
    int pulseWidth = 1000 + (throttle / 100.0) * 1000;
    ESC.writeMicroseconds(pulseWidth);

    // Give motor time to stabilize
    delay(stabilizationTime);

    // Collect thrust data
    SensorData vals = averageData(scale, WCS1, collectionTime);

    int amps = WCS1.A_DC();

    // Print CSV data
    Serial.print(throttle, 2);
    Serial.print(",");
    Serial.print(vals.averageThrust, 4);
    Serial.print(",");
    Serial.println(vals.averageCurrent, 3);
  }

  // Turn the motor off
  ESC.writeMicroseconds(1000);
  Serial.println("END OF DATA COLLECTION");
}


SensorData averageData(HX711 &scale, WCS WCS1, unsigned long duration) {
  SensorData results;
  float amps;
  float thrustSum = 0.0;
  float currentSum;
  unsigned long dataCount = 0;
  unsigned long startTime = millis();
  float weightReading;

  while (millis() - startTime < duration) {

    // Check if the HX711 has a new reading ready
    if (scale.is_ready()) {

      weightReading = scale.get_units(1);
      amps = WCS1.A_DC();

      thrustSum += weightReading;
      currentSum += amps;
      dataCount++;
    }
  }
  results.averageThrust = thrustSum / dataCount;
  results.averageCurrent = currentSum / dataCount;
  return results;
}
