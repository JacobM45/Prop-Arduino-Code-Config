#include <Servo.h>
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

// preload the functions here
float readLoadCellAverage(HX711 &scale, unsigned long duration);

void thrustTest(int iterations = 10,unsigned long collectionTime = 5000,float bottomRange = 0,float topRange = 100);

void setup() {

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  ESC.attach(9, 1000, 2000);  // (pin, min pulse width, max pulse width)
  ESC.writeMicroseconds(1000);

  //scale.begin(21, 20);  // pin 21 and 20
  //scale.set_scale(calibrationFactor);
  //scale.tare();

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
    float thrust = 0; //readLoadCellAverage(scale, collectionTime);

    // Replace with current sensor reading
    int rawADC = analogRead(sensorIn); // Reads a value from 0 to 1023
  
    // 1. Convert the raw ADC reading into actual volts output by the sensor
    float sensorVoltage = (rawADC / 1023.0) * vccVoltage;
  
    // 2. Account for the zero-current baseline offset (usually VCC / 2, which is 2.5V)
    float zeroCurrentOffset = vccVoltage / 2.0;
    float netVoltage = sensorVoltage - zeroCurrentOffset;
  
    // 3. Convert the voltage into Amps (net voltage divided by sensitivity in Volts)
    float amps = (netVoltage * 1000.0) / mVperAmp; 

    // Print CSV data
    Serial.print(throttle, 2);
    Serial.print(",");
    Serial.print(thrust, 4);
    Serial.print(",");
    Serial.println(amps, 3);
  }

  // Turn the motor off
  ESC.writeMicroseconds(1000);
  Serial.println("END OF DATA COLLECTION");
}


float readLoadCellAverage(HX711 &scale, unsigned long duration) {

  float dataSum = 0.0;
  unsigned long dataCount = 0;

  unsigned long startTime = millis();

  while (millis() - startTime < duration) {

    // Check if the HX711 has a new reading ready
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