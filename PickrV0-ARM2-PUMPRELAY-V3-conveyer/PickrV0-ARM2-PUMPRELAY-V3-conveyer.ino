#include <MyActuator.h>
#include <ClearpathSD.h>
#include <avr/wdt.h>

MyActuator canbus = MyActuator();

// MOTORCOUNT includes z motor, excludes wheel motor
const int MOTORCOUNT = 4; 

// define pins
// SPI pins D11, D12, D13, 
// Teknic pins A0, A2, A3, D9
// Prox sensor D1
// GPIO D2, D3, D4, D6, D7, D8
const int HLFB = A3;
const int inputB = 10;
const int inputA = A2;
const int enable = A0;

// GPIO assignment
const int prox = A5;
const int VALVE = 8;
const int PUMP = 7;
const int CUTTER = 6;
const int CONVEYOR = 4;
const int STEPPERDIR = 3;
const int STEPPERMOVE = 2;

ClearpathSD zmotor = ClearpathSD(HLFB, inputA, inputB, enable, prox);

// Canbus motors CANIDs
int16_t RMDX8 = 0x141; // shoulder
int16_t RMDX6 = 0x143; // elbow
int16_t xg1 = 0x144; // wrist

// if RMDL is within 1 rotation of zero position, it will be correct
int32_t xg1zero = 0x0;
int32_t x8zero = 0x0;
int32_t x6zero = 0x0;

// multitasking 
long int listenInterval = 20;
long int motionInterval = 20;
long int listenTime = 0;
long int motionTime = 0;

// zmotor direction -1(down) or 1(up)
int zdirection = 1;
int32_t zpos = 0;
int32_t ztarget = 0;

// canbus motor positions
int32_t x8pos = 0;
int32_t x6pos = 0;
int32_t xg1pos = 0;

int16_t x8spd = 0;
int16_t x6spd = 0;
int16_t xg1spd = 0;

int32_t x8target = 0;
int32_t x6target = 0;
int32_t xg1target = 0;

bool x8inRange = false;
bool x6inRange = false;
bool xg1inRange = false;
bool zinRange = false;

int errorStatus = 0x0;

// Serial Comms
byte reply[MOTORCOUNT * 13];

// Operation states
// 1 = idle, 2 = moving
int runPickerState = 1; 

// FIFO queue
const int qsize = 100;
const int esize = 10;
byte queue[qsize][esize];
int front = 0;
int rear = -1;
int itemCount = 0;
byte element[10];

// loop time vars
unsigned long startMillis;
unsigned long currentMillis;
unsigned long period = 20;

// zmotor interrupt
// when calling absPos, zdirection must be set
// when calling absPos, ztarget must also be set
// when homing, set desiredzpos
ISR(TIMER1_OVF_vect){
  zpos = zpos + zdirection;
  if (zpos == ztarget){
    zmotor.stop();
  }
}

void setup() {
  Serial.begin(115200);

    // set pinMode for GPIO pins
  pinMode(STEPPERMOVE, OUTPUT);
  pinMode(STEPPERDIR, OUTPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(CUTTER, OUTPUT);
  pinMode(VALVE, OUTPUT);
  pinMode(CONVEYOR, OUTPUT);
//  pinMode(prox, INPUT);
  
  digitalWrite(PUMP, HIGH);
  digitalWrite(VALVE, HIGH);
  digitalWrite(CONVEYOR, HIGH);  
  digitalWrite(STEPPERDIR, LOW);
  digitalWrite(STEPPERMOVE, LOW);
  digitalWrite(CUTTER, LOW);  
  
  // Start watchdog timer
//  wdt_disable();
//  delay(3000);
//  wdt_enable(WDTO_500MS);

  // Start CANBUS
  canbus.begin();

  // shutdown all motors
  canbus.shutdown(RMDX8);
  canbus.checkReceived(0);
  canbus.shutdown(RMDX6);
  canbus.checkReceived(0);
  canbus.shutdown(xg1);
  canbus.checkReceived(0);

  // loop time
  startMillis = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Tasks
  // Read 64 byte serial buffer
  // Filter by instructions
  // if Move command, append to move buffer
  // if Canbus command, execute immediately
  // Check speed and positions of all motors
  // execute all move commands when ready (motors in range and not moving)
  // listen Serial every serial interval
  canbus.checkReceived(0);
  canbus.checkReceived(0);
  canbus.checkReceived(0);
  canbus.checkReceived(0);

  // run trackmotion every 100 ms instead of every loop 
  // to minimize canbus traffic

  currentMillis = millis();
  if (currentMillis - startMillis >= period){
    trackMotion();    
    startMillis = currentMillis;
  }
  
  listenSerial();

  // reset watchdog timer every loop
//  wdt_reset();
}

void beginMotion(){
  // start move commands
  int32_t movePkg[MOTORCOUNT + 1][4];
  
  // check if queue is empty
  if (itemCount != 0 && runPickerState == 2){
    
    // execute MOTORCOUNT move commands from move queue
    for (int i = 0; i < MOTORCOUNT + 1; i++){
     
      // get move command from move queue
      dequeue();

      if (element != NULL){
        int16_t motorID = 0x0;
        motorID = motorID | (int16_t)element[0];
        motorID = motorID | ((int16_t)element[1] << 8); 
        
        // Check if Teknic move or CANBUS move
        if (motorID == 0x140){
          if (element[2] == 0xA4){
            // Absolute Move
            // Move Teknic Motor
            // convert to desired z pos
            int32_t angleControl = 0x0;
            angleControl = angleControl | ((int32_t)element[6]);
            angleControl = angleControl | ((int32_t)element[7] << 8);
            angleControl = angleControl | ((int32_t)element[8] << 16);
            angleControl = angleControl | ((int32_t)element[9] << 24);
            
            int16_t speedControl = 0x0;
            speedControl = speedControl | ((int16_t)element[4]);
            speedControl = speedControl | ((int16_t)element[5] << 8);
            
            int16_t spdd = (double)speedControl / 360.0 * 60.0;
            zmotor.setSpeed(spdd);
            
            ztarget = (int32_t)((double)angleControl / 36000 / 19.113125 * 122324);
            zdirection = zmotor.setAbsPos(ztarget);

            movePkg[i][0] = 0;
            movePkg[i][1] = 0;
            movePkg[i][2] = 0;
            movePkg[i][3] = 0;
            
          } else if (element[2] == 0xA8){
            // Incremental Move
            // Move Teknic Motor
            // convert to desired z pos
            int32_t angleControl = 0x0;
            angleControl = angleControl | ((int32_t)element[6]);
            angleControl = angleControl | ((int32_t)element[7] << 8);
            angleControl = angleControl | ((int32_t)element[8] << 16);
            angleControl = angleControl | ((int32_t)element[9] << 24);
            
            int16_t speedControl = 0x0;
            speedControl = speedControl | ((int16_t)element[4]);
            speedControl = speedControl | ((int16_t)element[5] << 8);
            
            int16_t spdd = (double)speedControl / 360.0 * 60.0;
            zmotor.setSpeed(spdd);
            
            ztarget = zpos + (int32_t)((double)angleControl / 36000.0 / 19.113125 * 122324.0);
            zdirection = zmotor.setAbsPos(ztarget);

            movePkg[i][0] = 0;
            movePkg[i][1] = 0;
            movePkg[i][2] = 0;
            movePkg[i][3] = 0;
          } 
        } else { 
          // Move canbus Motor
          // filter by abs or incremental move
          if (element[2] == 0xA4){
            // Absolute Move
            int32_t angleControl = 0x0;
            angleControl = angleControl | ((int32_t)element[6]);
            angleControl = angleControl | ((int32_t)element[7] << 8);
            angleControl = angleControl | ((int32_t)element[8] << 16);
            angleControl = angleControl | ((int32_t)element[9] << 24);
            
            int16_t maxSpeed = 0x0;
            maxSpeed = maxSpeed | ((int16_t)element[4]);
            maxSpeed = maxSpeed | ((int16_t)element[5] << 8);

            if (motorID == RMDX8) {
              x8target = angleControl + x8zero;
            } else if (motorID == RMDX6){
              x6target = angleControl + x6zero;
            } else if (motorID == xg1){
              xg1target = angleControl + xg1zero;
            }

            int32_t angleAdjusted = 0x0;
            if (motorID == xg1){
              angleAdjusted = angleControl + xg1zero;
              
              movePkg[i][0] = 0xA4;
              movePkg[i][1] = xg1;
              movePkg[i][2] = angleAdjusted;
              movePkg[i][3] = maxSpeed;
              
//              canbus.setAbsPos(motorID, angleAdjusted, maxSpeed);
//              canbus.checkReceived(0);
            }else if (motorID == RMDX8){
              angleAdjusted = angleControl + x8zero;
              
              movePkg[i][0] = 0xA4;
              movePkg[i][1] = RMDX8;
              movePkg[i][2] = angleAdjusted;
              movePkg[i][3] = maxSpeed;
              
//              canbus.setAbsPos(motorID, angleAdjusted, maxSpeed);
//              canbus.checkReceived(0);
            } else if (motorID == RMDX6){
              angleAdjusted = angleControl + x6zero;

              movePkg[i][0] = 0xA4;
              movePkg[i][1] = RMDX6;
              movePkg[i][2] = angleAdjusted;
              movePkg[i][3] = maxSpeed;
              
//              canbus.setAbsPos(motorID, angleAdjusted, maxSpeed);
//              canbus.checkReceived(0);
            }
            
          } else if (element[2] == 0xA8) {
            // incremental Move
            int32_t angleControl = 0x0;
            angleControl = angleControl | ((int32_t)element[6]);
            angleControl = angleControl | ((int32_t)element[7] << 8);
            angleControl = angleControl | ((int32_t)element[8] << 16);
            angleControl = angleControl | ((int32_t)element[9] << 24);
            
            int16_t maxSpeed = 0x0;
            maxSpeed = maxSpeed | ((int16_t)element[4]);
            maxSpeed = maxSpeed | ((int16_t)element[5] << 8);

            if (motorID == RMDX8) {
              x8target = x8pos - x8zero + angleControl;
            } else if (motorID == RMDX6){
              x6target = x6pos - x6zero + angleControl;
            } else if (motorID == xg1){
              xg1target = xg1pos - xg1zero + angleControl;
            }

            movePkg[i][0] = 0xA8;
            movePkg[i][1] = motorID;
            movePkg[i][2] = angleControl;
            movePkg[i][3] = maxSpeed;
            
//            canbus.setIncrementalPos(motorID, angleControl, maxSpeed);
//            canbus.checkReceived(0);
          } else if (element[2] == 0xA6){
            
            // GPIO OUTPUT
            // element[0] = ID
            // element[1] = ID
            // element[2] = ??
            // element[3] = ??
            // element[4] = ??
            // element[5] = ??
            // element[6] = ??
            // element[7] = ??
            // element[8] = ??
            // element[9] = ??

            // Vacuum Relay, element[9], GROUND = ON
            if (element[9] == 0x00){
              // pump off
              // valve off
              digitalWrite(PUMP, HIGH);
              digitalWrite(VALVE, HIGH);
            } else if (element[9] == 0x01){
              // pump on
              // valve off
              digitalWrite(PUMP, LOW);
              digitalWrite(VALVE, HIGH);
            } else if (0x10){
              // pump off
              // valve on
              digitalWrite(PUMP, HIGH);
              digitalWrite(VALVE, LOW);
            } else if (0x11){
              // pump on
              // valve on
              digitalWrite(PUMP, LOW);
              digitalWrite(VALVE, LOW);
            }
          } 
        }
      }
    }

    // run all packaged moves
    for (int i = 0; i < MOTORCOUNT + 1; i++){
      
//      Serial.print(movePkg[i][0]);
//      Serial.print(" ");
//      Serial.print(movePkg[i][1]);
//      Serial.print(" ");
//      Serial.print(movePkg[i][2]);
//      Serial.print(" ");
//      Serial.print(movePkg[i][3]);
//      Serial.println();
            
      if (movePkg[i][0] == 0xA8){
        // incremental move
        canbus.setIncrementalPos(movePkg[i][1], movePkg[i][2], movePkg[i][3]);
        canbus.checkReceived(0);     
      }else if (movePkg[i][0] == 0xA4){
        // absolute move
        canbus.setAbsPos(movePkg[i][1], movePkg[i][2], movePkg[i][3]);
        canbus.checkReceived(0);
      }
    }
  } else {
    // if queue is empty there all moves are complete
    runPickerState = 1; 
  }
}

// check for errors
void errorCheck(){
  
}

// track position of all motors
// if motion is complete, start next moves
// if all moves are complete, send move complete flag
void trackMotion(){
  // reset pos
  x8pos = 0;
  x6pos = 0;
  xg1pos = 0;

  // reset speeds
  x8spd = 0;
  x6spd = 0;
  xg1spd = 0;

  // reset inRange
  x8inRange = false;
  x6inRange = false;
  xg1inRange = false;

  int64_t canReplyRMDX8 = 0x0;
  canbus.getAbsAngle(RMDX8);
  canReplyRMDX8 = canbus.checkReceived(0);
  x8pos = x8pos | (int32_t)((canReplyRMDX8 & 0xFF000000) >> 24);
  x8pos = x8pos | (int32_t)((canReplyRMDX8 & 0x00FF0000) >> 8);
  x8pos = x8pos | (int32_t)((canReplyRMDX8 & 0x0000FF00) << 8);
  x8pos = x8pos | (int32_t)((canReplyRMDX8 & 0x000000FF) << 24);

  int64_t canReplyRMDX6 = 0x0;
  canbus.getAbsAngle(RMDX6);
  canReplyRMDX6 = canbus.checkReceived(0);
  x6pos = x6pos | (int32_t)((canReplyRMDX6 & 0xFF000000) >> 24);
  x6pos = x6pos | (int32_t)((canReplyRMDX6 & 0x00FF0000) >> 8);
  x6pos = x6pos | (int32_t)((canReplyRMDX6 & 0x0000FF00) << 8);
  x6pos = x6pos | (int32_t)((canReplyRMDX6 & 0x000000FF) << 24);

  int64_t canReplyxg1 = 0x0;
  // get Abs angle 
  // encoder only rotates 1 full revolution
  // reply: 142 92 xx xx FF FF FF FF FF
  canbus.getAbsAngle(xg1);
  canReplyxg1 = canbus.checkReceived(0);
  xg1pos = xg1pos | (int32_t)((canReplyxg1 & 0xFF000000) >> 24);
  xg1pos = xg1pos | (int32_t)((canReplyxg1 & 0x00FF0000) >> 8);
  xg1pos = xg1pos | (int32_t)((canReplyxg1 & 0x0000FF00) << 8);
  xg1pos = xg1pos | (int32_t)((canReplyxg1 & 0x000000FF) << 24);

  int32_t range = 200;

  // check if motors are in range of position
  if (x8pos >= x8target - range && x8pos <= x8target + range){
    x8inRange = true;
  } else {
    x8inRange = false;
  }

  // check if motors are in range of position
  if (x6pos >= x6target - range && x6pos <= x6target + range){
    x6inRange = true; 
  } else {
    x6inRange = false;
  }

  // check if motors are in range of position
  if (xg1pos >= xg1target - 1000 && xg1pos <= xg1target + 1000){
    xg1inRange = true;
  } else {
    xg1inRange = false;
  }

  zinRange = (zpos == ztarget);
  bool inRange = x8inRange && x6inRange && xg1inRange && zinRange;
    
  // start next set of moves if the previous ones have been completed
  
  if (inRange && runPickerState == 2 ){
    beginMotion();
  }
}

// Listen Serial
// Read 64 bytes from serial buffer 
// extract motorID and commandID
// filter by move command and non move command
// send non move commands
// store move commands
void listenSerial(){
  // if serial buffer contains a CANBUS msg
  int msgs = 0;
  byte sbuf[10];
  byte mbuf[(MOTORCOUNT + 1) * 10];
  
  if (Serial.available() > 9){

    msgs = Serial.available()/10;
    
    for (int i = 0; i < 10; i++){
      sbuf[i] = Serial.read();
    }

    // for each available message, store it and send to CAN
    // can support up to 6 serial messages per loop
    // Serial message format = 10 bytes 
    // ID followed by 8 bytes of can message
    // Read all available bytes in serial buffer    

    // check if 60 bytes is a move command
    // 60 bytes MUST BE 6 move commands
    // A4 = Absolute Move
    // A8 = Incremental Move
    // A6 = GPIO moves (turn on solenoid valve)
    if (sbuf[2] == 0xA4 || sbuf[2] == 0xA8 || sbuf[2] == 0xA6){
      // baud rate = 115200
      // 14400 bytes per 1000ms
      // 1000ms / 14400 bytes * 60 = 4.166ms transmit time
      
      long int startWait = millis();

      while (Serial.available() < (MOTORCOUNT)*10){
        // if enough bytes received, proceed
        long int waitTime = millis();        
        // if 100ms have elapsed break loop
        if (waitTime - startWait > 100){
          break;
        }
      }
      
      for (int i = 0; i < (MOTORCOUNT)*10; i++){
        mbuf[i] = Serial.read();
      }
      
      byte mmsg[10]= {0,0,0,0,0,0,0,0,0,0};
      
      mmsg[0] = sbuf[0];
      mmsg[1] = sbuf[1];
      mmsg[2] = sbuf[2];
      mmsg[3] = sbuf[3];
      mmsg[4] = sbuf[4];
      mmsg[5] = sbuf[5];
      mmsg[6] = sbuf[6];
      mmsg[7] = sbuf[7];
      mmsg[8] = sbuf[8];
      mmsg[9] = sbuf[9];

      enqueue(mmsg);
      
      for(int i = 0; i < MOTORCOUNT; i++){
        mmsg[0] = mbuf[10*i];
        mmsg[1] = mbuf[10*i + 1];
        mmsg[2] = mbuf[10*i + 2];
        mmsg[3] = mbuf[10*i + 3];
        mmsg[4] = mbuf[10*i + 4];
        mmsg[5] = mbuf[10*i + 5];
        mmsg[6] = mbuf[10*i + 6];
        mmsg[7] = mbuf[10*i + 7];
        mmsg[8] = mbuf[10*i + 8];
        mmsg[9] = mbuf[10*i + 9];

        enqueue(mmsg);
      }
      // Acknowledge Move Commands
      byte ackStatus = 0;
      byte ackError = 0;
      byte ackGPO = 0;
      byte ackGPI = 0;
      byte ack[8] = {0xAA, 0xBB, 0xCC, 0xDD, ackGPO, ackGPI, ackStatus, ackError};

//      Serial.write(ack, 8);
      
    }else{
      // if not move command
      // read 10 bytes at a time
      for (int i = 0; i < msgs; i++){
        byte cmsg[8] = {0,0,0,0,0,0,0,0};
        int16_t motorID = 0x0;
        
        // extract motor ID
        motorID = motorID | (int16_t)sbuf[10*i];
        motorID = motorID | ((int16_t)sbuf[10*i + 1] << 8);
        
        // pkg can message into 8 byte array
        cmsg[0] = sbuf[10*i + 2];
        cmsg[1] = sbuf[10*i + 3];
        cmsg[2] = sbuf[10*i + 4];
        cmsg[3] = sbuf[10*i + 5];
        cmsg[4] = sbuf[10*i + 6];
        cmsg[5] = sbuf[10*i + 7];
        cmsg[6] = sbuf[10*i + 8];
        cmsg[7] = sbuf[10*i + 9];  

        translateAndSend(motorID, cmsg);
      }
    }
    // clear serial buffer after read is done
    for (int i = 0; i < 64; i++){
      Serial.read();
    }
  }
}

// get current position of all motors and save as new zero position
void homeMotors(){
  trackMotion();
  if (runPickerState == 1){
    x8zero = x8pos;
    x6zero = x6pos;
    xg1zero = xg1pos;
    zmotor.home();
    zpos = 0;
  }
}

// Automatic Homing
void autoHomeMotor(int16_t homingMotorID){
  int16_t torqueCurrentRaw = 0x0;
  int16_t torqueCurrentAvg = 0x0;

  // Serial.print for ctrl f
  int printreply = 0;
  int16_t tqcWindow[10] = {0,0,0,0,0,0,0,0,0,0};
  
  // filter by canid (special case for z motor)
  if (homingMotorID == 0x143 || homingMotorID == 0x141){
    bool homed = false;
    
    // move towards home position
    canbus.setSpeed(homingMotorID, -1500);
    canbus.checkReceived(printreply);
    // wait for torque to stabilize
    delay(500);

    // rough primary home
    while (!homed){
      // get torque current
      canbus.getMotorStatus2(homingMotorID);
      int64_t homeReply = canbus.checkReceived(printreply);
      torqueCurrentRaw = 0x0;
      torqueCurrentRaw = torqueCurrentRaw | (int16_t)((homeReply & 0x0000FF0000000000) >> 40);
      torqueCurrentRaw = torqueCurrentRaw | (int16_t)((homeReply & 0x000000FF00000000) >> 24);
      
      // moving average for torque current
      // 0 is most recent
      tqcWindow[9] = tqcWindow[8];
      tqcWindow[8] = tqcWindow[7];
      tqcWindow[7] = tqcWindow[6];
      tqcWindow[6] = tqcWindow[5];
      tqcWindow[5] = tqcWindow[4];
      tqcWindow[4] = tqcWindow[3];
      tqcWindow[3] = tqcWindow[2];
      tqcWindow[2] = tqcWindow[1];
      tqcWindow[1] = tqcWindow[0];
      tqcWindow[0] = torqueCurrentRaw;   

      torqueCurrentAvg = (tqcWindow[0] + tqcWindow[1] + 
      tqcWindow[2] + tqcWindow[3] + tqcWindow[4] + 
      tqcWindow[5] + tqcWindow[6] + tqcWindow[7] + 
      tqcWindow[8] + tqcWindow[9]) / 10;

      // if torque current spikes, home position has been reached
      if (torqueCurrentAvg < -300){
        // reset moving average window
        for (int i = 0; i < 10; i++){
          tqcWindow[i] = 0;
        }
        // stop motor
        canbus.stop(homingMotorID);
        canbus.checkReceived(printreply);
        delay(500);
        // move 5 deg and rehome slower for greater precision
        canbus.setIncrementalPos(homingMotorID, 500, 10);
        canbus.checkReceived(printreply);
        delay(1000);
        // rehome but slower
        canbus.setSpeed(homingMotorID, -100);
        canbus.checkReceived(printreply);
        delay(500);
        while(!homed){
          // get torque current
          canbus.getMotorStatus2(homingMotorID);
          int64_t homeReply = canbus.checkReceived(printreply);
          torqueCurrentRaw = 0x0;
          torqueCurrentRaw = torqueCurrentRaw | (int16_t)((homeReply & 0x0000FF0000000000) >> 40);
          torqueCurrentRaw = torqueCurrentRaw | (int16_t)((homeReply & 0x000000FF00000000) >> 24);
    
          // moving average for torque current
          // 0 is most recent
          tqcWindow[9] = tqcWindow[8];
          tqcWindow[8] = tqcWindow[7];
          tqcWindow[7] = tqcWindow[6];
          tqcWindow[6] = tqcWindow[5];
          tqcWindow[5] = tqcWindow[4];
          tqcWindow[4] = tqcWindow[3];
          tqcWindow[3] = tqcWindow[2];
          tqcWindow[2] = tqcWindow[1];
          tqcWindow[1] = tqcWindow[0];
          tqcWindow[0] = torqueCurrentRaw;   
    
          torqueCurrentAvg = (tqcWindow[0] + tqcWindow[1] + 
          tqcWindow[2] + tqcWindow[3] + tqcWindow[4] + 
          tqcWindow[5] + tqcWindow[6] + tqcWindow[7] + 
          tqcWindow[8] + tqcWindow[9]) / 10;
      
          if (torqueCurrentAvg < -300){
            canbus.stop(homingMotorID);
            canbus.checkReceived(printreply);
            delay(100);
            canbus.setIncrementalPos(homingMotorID, 0, 10);
            canbus.checkReceived(printreply);
            delay(100);
            
            // get motor position
            canbus.getAbsAngle(homingMotorID);
            int64_t absAngle = canbus.checkReceived(printreply);
            int32_t edgePos = 0x0;
            edgePos = edgePos | (int32_t)((absAngle & 0xFF000000) >> 24);
            edgePos = edgePos | (int32_t)((absAngle & 0x00FF0000) >> 8);
            edgePos = edgePos | (int32_t)((absAngle & 0x0000FF00) << 8);
            edgePos = edgePos | (int32_t)((absAngle & 0x000000FF) << 24);
    
            // set zero
            if (homingMotorID == 0x141){
              x8zero = edgePos + 8120;
            } else if (homingMotorID == 0x143) {
              x6zero = edgePos + 14395;
            }
            homed = true;
          }
        }
      }
    }    
  }
}

//void homeZmotor(){
//  zmotor.engage();
//  zmotor.setSpeed(100);
//  // IMPORTANT: make sure z axis is below proximity sensor before 
//  // running auto home
//  // move up 
//  ztarget = 200000;
//  zdirection = zmotor.setAbsPos(ztarget);
//
//  bool proxWindow[5] = {0,0,0,0,0};
//
//  // detect rising edge
//  bool risingEdgeDetected = false;
//  
//  // reset prox window
//  for (int i = 0; i < 5; i++){
//    proxWindow[i] = 0;
//  }
//
//  while(!risingEdgeDetected){
//    // 0 is most recent
//    proxWindow[4] = proxWindow[3];
//    proxWindow[3] = proxWindow[2];
//    proxWindow[2] = proxWindow[1];
//    proxWindow[1] = proxWindow[0];
//    proxWindow[0] = digitalRead(prox);
//    
//    // check for rising edge
//    if (proxWindow[4] == 0 && proxWindow[3] == 0 && proxWindow[2] == 1
//    && proxWindow[1] == 1 && proxWindow[0] == 1){
//      
//      // rising edge detected
//      risingEdgeDetected = true;
//      zmotor.stop();
//      zpos = 0;
//      zmotor.setzpos(zpos);
//      zmotor.home();
//      delay(500);
//      
//      // move z motor to zero position
//      ztarget = 18000;
//      zdirection = zmotor.setAbsPos(ztarget);
//
//      // wait for move to complete
////      bool done = false;
////      int count = 0;
////      while(!done){
////        count++;
////        delay(1);
////        if (zpos == ztarget){
////          done = true;
////        }
////      }
////
////      // zero the z motor
////      zpos = 0;
////      zmotor.setzpos(zpos);
//      return;
//    }
//  }
//}

void translateAndSend(int16_t motorID, byte cmsg[]){
  
  byte commandID = cmsg[0];
  
  if (commandID == 0x9A){
    // poll motors
    pollMotors();
  }else if(commandID == 0xD1){
    // start move
    if (itemCount != 0){
      zmotor.engage();
      runPickerState = 2;
      beginMotion();
    }
  }else if(commandID == 0x63){
    // home
    homeMotors();
  }else if (commandID == 0x64){
    // auto home motors
    // zero wrist motor
    homeZmotor();
    // zero the z motor
    zpos = 0;
    zmotor.setzpos(zpos); 
    zmotor.home();
      
  }else if(commandID == 0x80){
    // shutdown
    canbus.shutdown(RMDX8);
    canbus.checkReceived(0);
    canbus.shutdown(RMDX6);
    canbus.checkReceived(0);
    canbus.shutdown(xg1);
    canbus.checkReceived(0);

    zmotor.stop();
    ztarget = zpos;
  }else if(commandID == 0x81){
    // stop
      canbus.stop(RMDX8);
      canbus.checkReceived(0);
      canbus.stop(RMDX6);
      canbus.checkReceived(0);
      canbus.stop(xg1);
      canbus.checkReceived(0);
      
      zmotor.stop();
      ztarget = zpos;
  }else if(commandID == 0x8B){
    // disengage
    // set picker state to idle
    
    zmotor.disengage();
    ztarget = zpos;
    
    // disengage all canbus motors
    canbus.shutdown(RMDX8);
    canbus.checkReceived(0);
    canbus.shutdown(RMDX6);
    canbus.checkReceived(0);
    canbus.shutdown(xg1);
    canbus.checkReceived(0);
    
  }else if(commandID == 0xAA){
    
    // Forward Move single pulse
    digitalWrite(STEPPERDIR, HIGH);
    delay(12);
    digitalWrite(STEPPERMOVE, HIGH);
    delay(18);
    digitalWrite(STEPPERMOVE, LOW);
    delay(12);
    digitalWrite(STEPPERDIR, LOW);
    
  }else if(commandID == 0xAB){
    
    // Backward Move
    digitalWrite(STEPPERDIR, LOW);
    delay(12);
    digitalWrite(STEPPERMOVE, HIGH);
    delay(18);
    digitalWrite(STEPPERMOVE, LOW);
    delay(12);
    digitalWrite(STEPPERDIR, LOW);
    
  }else if (commandID == 0xAD){
    
    // Actuate Conveyor for 3 seconds
    digitalWrite(CONVEYOR, LOW);
    delay(3000);
    digitalWrite(CONVEYOR, HIGH);
    
  }else if (commandID == 0xAF){
    // Actuate Cutter
    // digital write a square pulse 
    digitalWrite(CUTTER, HIGH);
    delay(36);
    digitalWrite(CUTTER, LOW);   

    // Actuate Conveyor
    digitalWrite(CONVEYOR, LOW);
    
  }else if(commandID == 0xD2){
    // clear move buffer
    clearQueue();
  }

  // Acknowledge reply
  byte ackStatus = 0;
  byte ackError = 0;
  byte ackGPO = 0;
  byte ackGPI = 0;
  byte ack[8] = {0xAA, 0xBB, 0xCC, 0xDD, ackGPO, ackGPI, ackStatus, ackError};
  
//  Serial.write(ack, 8);
}

// getMotorStatus1 and getMotorStatus2 for all motors
// 13 BYTES PER MOTOR
// get position and HLFB of teknic motor
// Write all 52 bytes to serial 
void pollMotors(){
  
  int16_t motorIDs[4] = {0x144, 0x143, 0x141};
  int sz = 3;

  // reset reply
  for (int i = 0; i < MOTORCOUNT * 13; i++){
    reply[i] = 0;
  }

  // for all motors
  for (int i = 0; i < sz; i++){

    int64_t status1 = 0;
    int64_t status2 = 0;
    int64_t posmsg = 0;
  
    // get motor status 1
    canbus.getMotorStatus1(motorIDs[i]);
    status1 = canbus.checkReceived(0);

    // special case RMD L 5015
    if (motorIDs[i] == 0x149){
      // package relevant bytes
      reply[13*i] = (byte)(motorIDs[i]);
      reply[13*i + 1] = (byte)(motorIDs[i] >> 8);
      reply[13*i + 2] = 0x01;
      reply[13*i + 3] = (byte)(status1>>8);
      reply[13*i + 4] = (byte)(status1);
    } else {
      // package relevant bytes
      reply[13*i] = (byte)(motorIDs[i]); //motorIDs
      reply[13*i + 1] = (byte)(motorIDs[i] >> 8); 
      reply[13*i + 2] = (byte)(status1>>32); // brk release
      reply[13*i + 3] = (byte)(status1>>8); // error status low
      reply[13*i + 4] = (byte)(status1); // error status high
    }
    
    // get motor status 2
    canbus.getMotorStatus2(motorIDs[i]);
    status2 = canbus.checkReceived(0);

    // package relevant bytes
    reply[13*i + 5] = (byte)(status2 >> 40); // torque current low
    reply[13*i + 6] = (byte)(status2 >> 32); // torque current high
    reply[13*i + 7] = (byte)(status2 >> 24); // motor speed low
    reply[13*i + 8] = (byte)(status2 >> 16); // motor speed high

    int32_t p = 0x0;
    if (motorIDs[i] == RMDX8){
      p = x8pos - x8zero;
    } else if (motorIDs[i] == RMDX6){
      p = x6pos - x6zero;
    } else if (motorIDs[i] == xg1){
      p = xg1pos - xg1zero;
    }
    
    reply[13*i + 9] = (byte)(p); // pos low 1
    reply[13*i + 10] = (byte)(p >> 8); // pos low 2
    reply[13*i + 11] = (byte)(p >> 16); // pos high 1
    reply[13*i + 12] = (byte)(p >> 24); // pos high 2
  }

  // get information from teknic motor
  reply[sz * 13] = 0x40;
  reply[sz * 13 + 1] = 0x01;
  reply[sz * 13 + 2] = 0x0;
  reply[sz * 13 + 3] = 0x0;
  reply[sz * 13 + 4] = 0x0;
  reply[sz * 13 + 5] = 0x0;
  reply[sz * 13 + 6] = 0x0;
  reply[sz * 13 + 7] = 0x0;
  reply[sz * 13 + 8] = 0x0;
  
  double zbuf = (double)zpos / 6400.0 * 36000.0;
  int32_t zposcdeg = (int32_t)zbuf;
  
  reply[sz * 13 + 9] = (byte)zposcdeg ;
  reply[sz * 13 + 10] = (byte)(zposcdeg >> 8);
  reply[sz * 13 + 11] = (byte)(zposcdeg >> 16);
  reply[sz * 13 + 12] = (byte)(zposcdeg >> 24);
  
  Serial.write(reply, MOTORCOUNT * 13);
}

void enqueue(byte element[esize]) {
  if(itemCount < qsize) {
    rear = (rear + 1) % qsize;
    for (int i = 0; i < esize; i++) {
      queue[rear][i] = element[i];
    }
    itemCount++;
  }
}

bool dequeue() {
  if (itemCount > 0) {
    for (int i = 0; i < esize; i++) {
      element[i] = queue[front][i];
    }
    
    front = (front + 1) % qsize;
    itemCount--;
    return true;
  } else {
    return false;
  }
}

bool isQueueFull() {
  return itemCount == qsize;
}

bool isQueueEmpty() {
  return itemCount == 0;
}

void clearQueue() {
  itemCount = 0;
  front = 0;
  rear = -1;
}

void printQueue() {
  int i = front;
  int count = 0;
  while (count < itemCount) {
    Serial.print("Element ");
    Serial.print(count);
    Serial.print(": ");
    for (int j = 0; j < esize; j++) {
      Serial.print(queue[i][j], HEX);
      Serial.print(" ");
    }
    Serial.println();
    i = (i + 1) % qsize;
    count++;
  }
}
