#include "Arduino.h"
#include "ClearpathSD.h"

ClearpathSD::ClearpathSD(int HL, int inpA, int inpB, int en, int pro){
  // define pins
  // standard HLFB = A3, inputA = A2, inputB = PD10, enable = A0;
  enable = en;
  inputA = inpA;
  inputB = inpB;
  HLFB = HL;
  prox = pro;
  
  // define pinmodes
  pinMode(en, OUTPUT);
  pinMode(inpA, OUTPUT);
  pinMode(inpB, OUTPUT);
  pinMode(HLFB, INPUT_PULLUP);
  pinMode(prox, INPUT);
	
  zpos = 0;  // 122,324 encoder counts per full range
  ppr = 6400;     // 6400 pulses per revolution
  zrpm = 100;   // default rpm = 100
  zaccel = 1000; // default acceleration 1000 rpm/s
}

// move up until prox sensor
// set pos to 122,324 encoder counts
// ppr = encoder counts per rev = 6400
// HOME PAUSES CODE EXECUTION
long int ClearpathSD::home(){ 
  // move up
  zpos = 0;
}

// absolute position move
// 0 encoder counts = lowest point
// 122,324 encoder counts = highest point 
// direction determined by math
int ClearpathSD::setAbsPos(long int pos){
  bool up;
  long int dist = 0;

  // calculate direction and distance
  // CW is up = inputA HIGH
  // CCW is down = inputA LOW
  if(pos > zpos){
    up = true; 
    dist = (pos - zpos) * ppr / 6400;

  }else if (pos < zpos){
	
    up = false;
    dist = (zpos - pos) * ppr / 6400;

  }else{
    return 0;
  };
	
	// set direction
	digitalWrite(inputA, up);

	// calculate pulses per second according to speed
	// calculate OCR1A and ICR1
	long int pps = zrpm * ppr / 60;
	long int setICR1 = 2000000.0/pps - 1;

	pulseGen(setICR1);

  zpos = pos;
  
  if (up){
	return 1;
  }else{
	return -1;
  }
}

// motion planning function
// calculate trapezoidal velocity curve based on 
// move distance, move speed, move acceleration
// set ICR1 to ramp and deramp at predetermined points
// return predetermined points and use interupt to set ICR1
// ClearpathSD::motionPlanning(int dist, int dir){
	
// }


// set Speed by changing pulse frequency
// max pulse freq = 667kHz
// min pulse period = 1.5us
// MCU clock = 16MHz --> no rpm limit
// motor RPM limit = 4000
ClearpathSD::setSpeed(int rpm){
  zrpm = rpm;
}

// engage motor by setting enable HIGH
ClearpathSD::engage(){
  // ensure motor is disabled
  digitalWrite(enable, LOW);

  // set inputs to low
  digitalWrite(inputA, LOW);
  digitalWrite(inputB, LOW);

  // toggle enable to engage motor
  digitalWrite(enable, HIGH);
}

// disengage motor
ClearpathSD::disengage(){
  stop();
  digitalWrite(enable, LOW);
  digitalWrite(inputA, LOW);
  digitalWrite(inputB, LOW);

}

// Stop motion and disable interrupt
ClearpathSD::stop(){
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
}

// Output pulses on digital pin 9 with set frequency
ClearpathSD::pulseGen(int setICR1){
  // Set up PWM
  TCCR1A = 0;
  TCCR1B = 0;
  
  TCCR1A |= (1<<WGM11) | (1<<COM1B1);
  // WGM13, WGM12, WGM11 = 1 for Fast PWM ICR1 Top
  // CS11 = 1 for 8 factor prescaling
  TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); 
  // Enable interrupts when timer1 overflows
  TIMSK1 |= (1<<TOIE1);
  // max pulse freq = 500khz
  OCR1B = 2;
  ICR1 = setICR1;
  	
}

ClearpathSD::setzpos(int zposTemp){
	zpos = zposTemp;
}