#include "ClearpathSD.h"

// define pins
const int HLFB = A3;
const int inputB = PD3;
const int inputA = A2;
const int enable = A0;

ClearpathSD zmotor = ClearpathSD(HLFB, inputA, inputB, enable);

// define rotation directions
// CW is up = inputA HIGH
// CCW is down = inputA LOW
const int up = 1;
const int down = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  zmotor.engage();
  zmotor.absMove(60000);
  delay(100);
  zmotor.absMove(100000);
  delay(100);
  zmotor.absMove(20000);
  delay(100);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
