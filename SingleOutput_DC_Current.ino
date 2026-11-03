#include <WinsonLib.h>

WCS WCS1 = WCS(0, _WCS1600);

double data = 0;

const int MAX_READINGS = 1000;
const int DURATION = 90000; //milliseconds
int initialTime = millis();
int time;

float currentData[MAX_READINGS]; 
int listSize = 0; 

void setup() {
  Serial.begin(9600);
  // Power-on Reset
  Serial.println("Reset");
  WCS1.Reset(); 
}

void loop() {
  data = WCS1.A_DC();
  time = millis() - initialTime;
  
  if (time < DURATION && sizeof(currentData) / sizeof(currentData[0]) < MAX_READINGS) {
    currentData[listSize] = (float)data;
    listSize++;
  }

  Serial.print("Current: ");
  Serial.println(String(data, 3) + " A");

  delay(DURATION/MAX_READINGS);
}
