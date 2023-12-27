#include "MyActuator.h"
#include "mcp2515_can.h"

// TODO
// invoices
// wiring for pickerv3
// MCU bug

#define CAN_CS_PIN 9   // Set the CAN chip select pin
mcp2515_can CAN(CAN_CS_PIN); // Create a CAN object

// constructor
MyActuator::MyActuator(){
  // motorID = ID;
}


// Start CAN Communication
MyActuator::begin(){
  while (CAN_OK != CAN.begin(CAN_1000KBPS)){
    delay(1000);
  }
}

// 1 Read PID Parameter Command (0x30)
// 0 - 255 units
// maximum range is set by motor model
MyActuator::getPIDParameters(uint16_t motorID){
  unsigned char buf[8] = {0x30, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 2 Write PID parameters to RAM Command (0x31)
// 0 - 255 units
// maximum range is set by motor model
MyActuator::setPIDParametersRAM(uint16_t motorID, uint8_t currKP, uint8_t currKI, uint8_t spdKP, uint8_t spdKI, uint8_t posKP, uint8_t posKI){
  unsigned char buf[8] = {0x31, 0, currKP, currKI, spdKP, spdKI, posKP, posKI};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 3 Write PID parameters to ROM Command (0x32)
// 0 - 255 units
// maximum range is set by motor model
MyActuator::setPIDParametersROM(uint16_t motorID, uint8_t currKP, uint8_t currKI, uint8_t spdKP, uint8_t spdKI, uint8_t posKP, uint8_t posKI){
  unsigned char buf[8] = {0x32, 0, currKP, currKI, spdKP, spdKI, posKP, posKI};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// Read acceleration command (0x42)
// 0 - 255 units
// maximum range is set by motor model
MyActuator::readAccel(uint16_t motorID){
  unsigned char buf[8] = {0x42, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 5 Write acceleration to RAM and ROM command (0x43)
// accel input = 1 degree per second per second / LSB
// accel range = 100 to 60000 dps/s
// index see RMD-X "Servo Motor Control" Datasheet
MyActuator::setAccel(uint16_t motorID, uint8_t index, uint32_t accel){
  uint8_t accel1 = (uint8_t)(accel);
  uint8_t accel2 = (uint8_t)(accel>>8);
  uint8_t accel3 = (uint8_t)(accel>>16);
  uint8_t accel4 = (uint8_t)(accel>>24);
  unsigned char buf[8] = {0x43, index, 0, 0, accel1, accel2, accel3, accel4};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 6 Read multi-turn encoder position data command (0x60)
// units in pulses
// 101580 pulses per revolution
MyActuator::getPos(uint16_t motorID){
  unsigned char buf[8] = {0x60, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 8 Read multi-turn encoder zero offset data command (0x62)
// units in pulses
// 101580 pulses per revolution
MyActuator::getZeroOffset(uint16_t motorID){
  unsigned char buf[8] = {0x62, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 9 Write encoder multi-turn value to ROM as motor zero command (0x63) 
// units in pulses
// 101580 pulses per revolution
MyActuator::setMotorZeroROM(uint16_t motorID, int32_t encoderOffset){
  // overflow should never happen. The arm will never be zerod there
  // convert from 0.01deg/LSB to pulses
  
  int8_t encoderOffset1 = (int8_t)(encoderOffset);
  int8_t encoderOffset2 = (int8_t)(encoderOffset>>8);
  int8_t encoderOffset3 = (int8_t)(encoderOffset>>16);
  int8_t encoderOffset4 = (int8_t)(encoderOffset>>24);
  
  unsigned char buf[8] = {0x63, 0, 0, 0, encoderOffset1, encoderOffset2, encoderOffset3, encoderOffset4};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// Write the current multi-turn position of the encoder to the ROM as the motor zero command (0x64)
MyActuator::setMotorZero(uint16_t motorID){
  unsigned char buf[8] = {0x64, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID, 0, 8, buf);
}

// 11 Read multi-turn angle command (0x92)
// units = 0.01deg/LSB
MyActuator::getAbsAngle(uint16_t motorID){
  unsigned char buf[8] = {0x92, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
  delay(10);
}

// 12 Read Motor Status 1 (0x9A)
// data1 = temperature
// data3 = brake release command
// data4 = voltage low byte
// data5 = voltage high byte
// data6 = error status low byte
// data7 = error status high byte
MyActuator::getMotorStatus1(uint16_t motorID){
  unsigned char buf[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 13 Read Motor Status 2 (0x9C)
// data1 = temperature
// data2 = torque current low byte
// data3 = torque current high byte
// data4 = motor speed low byte
// data5 = motor speed high byte
// data6 = motor angle low byte
// data7 = motor angle high byte
MyActuator::getMotorStatus2(uint16_t motorID){
  unsigned char buf[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 14 Read Motor Status 3 (0x9D)
// data1 = phase A current low byte
// data2 = phase A current high byte
// data1 = phase B current low byte
// data2 = phase B current high byte
// data1 = phase C current low byte
// data2 = phase C current high byte
MyActuator::getMotorStatus3(uint16_t motorID){
  unsigned char buf[8] = {0x9D, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 15 Motor shutdown command (0x80)
MyActuator::shutdown(uint16_t motorID){
  unsigned char buf[8] = {0x80, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 16 Motor stop command (0x81)
MyActuator::stop(uint16_t motorID){
  unsigned char buf[8] = {0x81, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 17 Torque closed-loop control command (0xA1)
// Input = torque current in Amps
// Control value = 0.01A/LSB
MyActuator::setTorque(uint16_t motorID, int16_t iqControl){
  uint8_t iqControl1 = (int8_t)(iqControl);
  uint8_t iqControl2 = (int8_t)(iqControl>>8); 
  unsigned char buf[8] = {0xA1, 0, 0, 0, iqControl1, iqControl2, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 18 Speed Closed-loop Control Command (0xA2)
// Input = speed in degrees per second
// Control value = 0.01dps/LSB
MyActuator::setSpeed(uint16_t motorID, int32_t speedControl){
  int8_t speedControl1 = (int8_t)(speedControl);
  int8_t speedControl2 = (int8_t)(speedControl >> 8);
  int8_t speedControl3 = (int8_t)(speedControl >> 16);
  int8_t speedControl4 = (int8_t)(speedControl >> 24);
  unsigned char buf[8] = {0xA2, 0, 0, 0, speedControl1, speedControl2, speedControl3, speedControl4};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 20 Absolute position closed-loop control command (0xA4)
// angleControl Input = position in degrees
// angleControl Control Value = 0.01deg/LSB
// maxSpeed Input = speed in degrees per second
// maxSpeed Control Value = speed in degrees per second
MyActuator::setAbsPos(uint16_t motorID, int32_t angleControl, int16_t maxSpeed){
  int8_t angleControl1 = (int8_t)(angleControl);
  int8_t angleControl2 = (int8_t)(angleControl >> 8);
  int8_t angleControl3 = (int8_t)(angleControl >> 16);
  int8_t angleControl4 = (int8_t)(angleControl >> 24);
  int8_t maxSpeed1 = (int8_t)(maxSpeed);
  int8_t maxSpeed2 = (int8_t)(maxSpeed>>8);
  unsigned char buf[8] = {0xA4, 0, maxSpeed1, maxSpeed2, angleControl1, angleControl2, angleControl3, angleControl4};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 21 Position tracking control command with speed limit (0xA5)
// angleControl Input = position in degrees
// angleControl Control Value = 0.01deg/LSB
// maxSpeed Input = speed in degrees per second
// maxSpeed Control Value = speed in degrees per second
MyActuator::setPosSpeed(uint16_t motorID, int32_t angleControl, int16_t maxSpeed){
  int8_t angleControl1 = (int8_t)(angleControl);
  int8_t angleControl2 = (int8_t)(angleControl >> 8);
  int8_t angleControl3 = (int8_t)(angleControl >> 16);
  int8_t angleControl4 = (int8_t)(angleControl >> 24);
  int8_t maxSpeed1 = (int8_t)(maxSpeed);
  int8_t maxSpeed2 = (int8_t)(maxSpeed>>8);
  unsigned char buf[8] = {0xA5, 0, maxSpeed1, maxSpeed2, angleControl1, angleControl2, angleControl3, angleControl4};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 22 Incremental position closed-loop control command (0xA8) 
// angleControl Input = position in degrees
// angleControl Control Value = 0.01deg/LSB
// maxSpeed Input = speed in degrees per second
// maxSpeed Control Value = speed in degrees per second
MyActuator::setIncrementalPos(uint16_t motorID, int32_t angleControl, int16_t maxSpeed){
  int8_t angleControl1 = (int8_t)(angleControl);
  int8_t angleControl2 = (int8_t)(angleControl >> 8);
  int8_t angleControl3 = (int8_t)(angleControl >> 16);
  int8_t angleControl4 = (int8_t)(angleControl >> 24);
  int8_t maxSpeed1 = (int8_t)(maxSpeed);
  int8_t maxSpeed2 = (int8_t)(maxSpeed>>8);
  unsigned char buf[8] = {0xA8, 0, maxSpeed1, maxSpeed2, angleControl1, angleControl2, angleControl3, angleControl4};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 23 System operating mode acquisition (0x70)
// data7 = operation loop mode
MyActuator::getMode(uint16_t motorID){
  unsigned char buf[8] = {0x70, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 24 Motor power acquisition (0x71)
// data6 = motor power low byte
// data7 = motor power high byte
MyActuator::getPower(uint16_t motorID){
  unsigned char buf[8] = {0x71, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 25 System reset command (0x76)
// motor will reset and not return command
MyActuator::resetProgram(uint16_t motorID){
  unsigned char buf[8] = {0x76, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 26. System brake release command (0x77)
MyActuator::brakeRelease(uint16_t motorID){
  unsigned char buf[8] = {0x77, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 27 System brake lock command (0x78)
// Lock held in ROM
MyActuator::brakeLock(uint16_t motorID){
  unsigned char buf[8] = {0x78, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 28 System runtime read command (0xB1)
// data4 = system run time low byte1
// data5 = system run time low byte2
// data6 = system run time low byte3
// data7 = system run time low byte4
MyActuator::runtime(uint16_t motorID){
  unsigned char buf[8] = {0xB1, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 29 System software version date read command (0xB2)
// data4 = version date low byte1
// data5 = version date low byte2
// data6 = version date low byte3
// data7 = version date low byte4
MyActuator::softwareVersion(uint16_t motorID){
  unsigned char buf[8] = {0xB2, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 30 Communication interruption protection time setting command (0xB3)
MyActuator::setProtectionTime(uint16_t motorID, uint32_t canRecvTime_MS){
  uint8_t canRecvTime_MS1 = (uint8_t)(canRecvTime_MS);
  uint8_t canRecvTime_MS2 = (uint8_t)(canRecvTime_MS>>8);
  uint8_t canRecvTime_MS3 = (uint8_t)(canRecvTime_MS>>16);
  uint8_t canRecvTime_MS4 = (uint8_t)(canRecvTime_MS>>24);
  unsigned char buf[8] = {0xB3, 0, 0, 0, canRecvTime_MS1, canRecvTime_MS2, canRecvTime_MS3, canRecvTime_MS4};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 31 Communication baud rate setting command (0xB4)
// BAUD rate change means the reply should be ignored
MyActuator::setBAUDRate(uint16_t motorID, uint8_t baudrate){
  unsigned char buf[8] = {0xB4, 0, 0, 0, 0, 0, 0, baudrate};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 32 Motor model reading command (0xB5)
// data1 to data7 = ASCII motor model
MyActuator::getModel(uint16_t motorID){
  unsigned char buf[8] = {0xB5, 0, 0, 0, 0, 0, 0, 0};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 33 (0x20) 33 Function control command 
MyActuator::functionControl(uint16_t motorID, uint8_t index, uint32_t input){
  uint8_t input1 = (uint8_t)(input);
  uint8_t input2 = (uint8_t)(input>>8);
  uint8_t input3 = (uint8_t)(input>>16);
  uint8_t input4 = (uint8_t)(input>>24);
  unsigned char buf[8] = {0x20, index, 0, 0, input1, input2, input3, input4};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
}

// 4. CANID setting command (0x79)
MyActuator::setCANID(uint16_t motorID, uint8_t CANID, uint8_t readWriteFlag){
  unsigned char buf[8] = {0x79, 0, 0, 0, 0, 0, 0, 6};
  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,buf);
//  motorID = CANID;
//  Serial.println(motorID,HEX);
//  Serial.println(CANID,HEX);
}

// 5. Motion Mode Control Command_CAN (0x400 + ID)
// Manufacturer recommends not using this command
MyActuator::motionModeControl(uint16_t motorID, uint16_t pdes, uint16_t vdes, uint16_t kp, uint16_t kd, uint16_t tff){
  
  uint16_t sendID = motorID - 0x140 + 0x400;
  
  if (pdes > 0xFFFF || vdes > 0xFFF || kp > 0xFFF || kd > 0xFFF || tff > 0xFFF){
	// Serial.print("Motion Mode Control input overflow");
  }else{
	  uint8_t d0 = pdes >> 8;
	  uint8_t d1 = pdes;
	  uint8_t d2 = vdes >> 4;
	  uint8_t d3 = ((vdes & 0x00F) << 4)| ((kp & 0xF00) >> 8) ;
	  uint8_t d4 = kp;
	  uint8_t d5 = kd >> 4;
	  uint8_t d6 = ((kd & 0x00F) << 4)| ((tff & 0xF00) >> 8);
	  uint8_t d7 = tff;
	  
	  // Serial.print(d0, HEX);
	  // Serial.print(" ");
	  // Serial.print(d1, HEX);
	  // Serial.print(" ");
	  // Serial.print(d2, HEX);
	  // Serial.print(" ");
	  // Serial.print(d3, HEX);
	  // Serial.print(" ");
	  // Serial.print(d4, HEX);
	  // Serial.print(" ");
	  // Serial.print(d5, HEX);
	  // Serial.print(" ");
	  // Serial.print(d6, HEX);
	  // Serial.print(" ");
	  // Serial.println(d7, HEX);
	  
	  unsigned char buf[8] = {d0, d1, d2, d3, d4, d5, d6, d7};
	  CAN.MCP_CAN::sendMsgBuf(sendID,0,8,buf);
  }
}


// Check if data is received, read, print it and return String
// IMPORTANT: The minimum delay between sending and receiving a message is 1ms
// This function also deletes the received message from the top of the stack

MyActuator::checkReceived(bool print, byte canReply[]){
  unsigned long id;
  uint8_t  type; // bit0: ext, bit1: rtr
  uint8_t  len;
  byte cd[8];
  
  long int startTime = millis();
  
  while(CAN_MSGAVAIL != CAN.checkReceive()){
	long int loopTime = millis();
	if (loopTime - startTime > 2){
	  return -1;
	}
  }
  
  
  if(CAN_MSGAVAIL == CAN.checkReceive()){ 
    // if message received read it based on ID

    CAN.readMsgBufID(&id, &len, cd);
	
	if(print){
		Serial.print("Reply: ");
		Serial.print(id, HEX);
		
		for (int i = 0; i < 8; i++){
		  Serial.print(" ");
		  Serial.print(cd[i], HEX);
		}
		Serial.println(" ");
	};
	
	canReply[0] = (byte)id;
	canReply[1] = (byte)(id >> 8);
	canReply[2] = cd[0];
	canReply[3] = cd[1];
	canReply[4] = cd[2];
	canReply[5] = cd[3];
	canReply[6] = cd[4];
	canReply[7] = cd[5];
	canReply[8] = cd[6];
	canReply[9] = cd[7];
	
  }
}

// Check if data is received, read, print it and return String
// IMPORTANT: The minimum delay between sending and receiving a message is 1ms
// This function also deletes the received message from the top of the stack

uint64_t MyActuator::checkReceived(bool print){
  unsigned long id;
  uint8_t  type; // bit0: ext, bit1: rtr
  uint8_t  len;
  byte cd[8];
  
  long int startTime = millis();
  while(CAN_MSGAVAIL != CAN.checkReceive()){
	long int loopTime = millis();
	if (loopTime - startTime > 2){
	  return 0;
	}
  }
  
  if(CAN_MSGAVAIL == CAN.checkReceive()){ 
    // if message received read it based on ID

    CAN.readMsgBufID(&id, &len, cd);

	if(print){
		Serial.print("Reply: ");
		Serial.print(id, HEX);
		Serial.print(" ");
		
		Serial.print("Length: ");
		Serial.print(len);
	
		for (int i = 0; i < 8; i++){
		  Serial.print(" ");
		  Serial.print(cd[i], HEX);
		}
		Serial.println(" ");
	}

	uint64_t fullCANMSG = 0x0;
    fullCANMSG = fullCANMSG | (uint64_t)cd[0]<<56;
    fullCANMSG = fullCANMSG | (uint64_t)cd[1]<<48;
    fullCANMSG = fullCANMSG | (uint64_t)cd[2]<<40;
    fullCANMSG = fullCANMSG | (uint64_t)cd[3]<<32;
    fullCANMSG = fullCANMSG | (uint64_t)cd[4]<<24;
    fullCANMSG = fullCANMSG | (uint64_t)cd[5]<<16;
    fullCANMSG = fullCANMSG | (uint64_t)cd[6]<<8;
    fullCANMSG = fullCANMSG | (uint64_t)cd[7];

    return fullCANMSG;  
    
  }
}

MyActuator::sendUnidCANMessage(uint16_t motorID, unsigned char cmsg[]){

  CAN.MCP_CAN::sendMsgBuf(motorID,0,8,cmsg);
}


/*switch(cd[0]){
    // 01  (0x30) read PID parameter command
    case 0x30:
      
      break;
    // 02  (0x31) Write PID parameters to RAM command
    case 0x31:
      
      break;
    // 03  (0x32) Write PID parameters to ROM command
    case 0x32:
      
      break;
    // 05  (0x43) Write acceleration to RAM and ROM command
    case 0x43:

      break;
    // 06  (0x60) Read multi-turn encoder position data command
    case 0x60:

      break;
    // 08 (0x62) Read multi-turn encoder zero offset data command
    case 0x62:
    
      break;
    // 09 (0x63) Write encoder multi-turn value to ROM as motor zero command
    case 0x63:

      break;
    // 11 (0x92) Read multi-turn angle command
    case 0x92:

      break;
    // 12 (0x9A) Read Motor Status 1 and Error Flag Command
    case 0x9A:
	  double v = (double)(((uint16_t)cd[4] | (uint16_t)cd[5] << 8)) / 10;
	  uint16_t e = (uint16_t)cd[6] | (uint16_t)cd[7] << 8;
      Serial.print(id - 0x100, HEX);
	  Serial.print(" Temp: ");
	  Serial.print(cd[1]);
	  Serial.print(" BR ");
	  Serial.print(cd[3]);
	  Serial.print(" V: ");
	  Serial.print(v);
	  Serial.print(" ErrStatus: ");
	  Serial.print(e, HEX);
      break;
    // 13 (0x9C) Read Motor Status 2 command
    case 0x9C:
      
      break;
    // 14 (0x9D) Read Motor Status 3 Command  
    case 0x9D:

      break;
    // 15 (0x80) Motor shutdown command
    case 0x80:

    break;   
    // 16 (0x81) Motor stop command     
    case 0x81:
	  Serial.print(id - 0x100, HEX);
	  Serial.println(" Stopped");
      break;
    // 17 (0xA1) Torque closed-loop control command
    case 0xA1:
    
      break;
    // 18 (0xA2) Speed Closed-loop Control Command 
    case 0xA2:

      break;
    // 20 (0xA4) Absolute position closed-loop control command
    case 0xA4:

      break;
    // 21 (0xA5) Position tracking control command with speed limit
    case 0xA5:

      break;
    // 22 (0xA8) Incremental position closed-loop control command 
    case 0xA8:
    
      break;
    // 23 (0x70) System operating mode acquisition
    case 0x70:

      break;
    // 24 (0x71) Motor power acquisition
    case 0x71:

      break;  
    // 25 (0x76) System reset command
    case 0x76:
      
      break;  
    // 26 (0x77) System brake release command
    case 0x77:

      break;  
    // 27 (0x78) System brake lock command
    case 0x78:

      break;  
    // 28 (0xB1) System runtime read command
    case 0xB1:

      break; 
    // 29 (0xB2)System software version date read command
    case 0xB2:

      break; 
    // 30 (0xB3) Communication interruption protection time setting command
    case 0xB3:

      break; 
    // 31 (0xB4) Communication baud rate setting command
    case 0xB4:

      break; 
    // 32 (0xB5) Motor model reading command
    case 0xB5:

      break; 
  }*/