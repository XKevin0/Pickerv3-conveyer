#ifndef MYACTUATOR_H
#define MYACTUATOR_H

#include <Arduino.h>
#include "mcp2515_can.h"
#include "MyActuator.h"

/*
	Make sure there is a small delay between sending CAN messages also
*/

class MyActuator {
  public: 
    MyActuator(); //constructor
    begin();  // begin CAN communication
    getPIDParameters(uint16_t motorID);   // 01  (0x30) Read PID parameter command
    setPIDParametersRAM(uint16_t motorID, uint8_t currKP, uint8_t currKI, uint8_t spdKP, uint8_t spdKI, uint8_t posKP, uint8_t posKI);  // 02  (0x31) Write PID parameters to RAM command
    setPIDParametersROM(uint16_t motorID, uint8_t currKP, uint8_t currKI, uint8_t spdKP, uint8_t spdKI, uint8_t posKP, uint8_t posKI);  // 03  (0x32) Write PID parameters to ROM command
    readAccel(uint16_t motorID); // 04 Read acceleration command (0x42)
	setAccel(uint16_t motorID, uint8_t index, uint32_t accel);             // 05  (0x43) Write acceleration to RAM and ROM command
    getPos(uint16_t motorID);        // 06  (0x60) Read multi-turn encoder position data command  
    getZeroOffset(uint16_t motorID); // 08 (0x62) Read multi-turn encoder zero offset data command
    setMotorZeroROM(uint16_t motorID, int32_t encoderOffset); // 09 (0x63) Write encoder multi-turn value to ROM as motor zero command
    setMotorZero(uint16_t motorID); // 10 Write the current multi-turn position of the encoder to the ROM as the motor zero command (0x64)
	getAbsAngle(uint16_t motorID); // 11 (0x92) Read multi-turn angle command
    getMotorStatus1(uint16_t motorID); // 12 (0x9A) Read Motor Status 1 and Error Flag Command
    getMotorStatus2(uint16_t motorID);   // 13 (0x9C) Read Motor Status 2 command
    getMotorStatus3(uint16_t motorID); // 14 (0x9D) Read Motor Status 3 Command  
    shutdown(uint16_t motorID); // 15 (0x80) Motor shutdown command 
    stop(uint16_t motorID); // 16 (0x81) Motor stop command  
    setTorque(uint16_t motorID, int16_t iqControl); // 17 (0xA1) Torque closed-loop control command
    setSpeed(uint16_t motorID, int32_t speedControl); // 18 (0xA2) Speed Closed-loop Control Command 
    setAbsPos(uint16_t motorID, int32_t angleControl, int16_t maxSpeed); // 20 (0xA4) Absolute position closed-loop control command
    setPosSpeed(uint16_t motorID, int32_t angleControl, int16_t maxSpeed); // 21 (0xA5) Position tracking control command with speed limit 
    setIncrementalPos(uint16_t motorID, int32_t angleControl, int16_t maxSpeed); // 22 (0xA8) Incremental position closed-loop control command 
    getMode(uint16_t motorID); // 23 (0x70) System operating mode acquisition 
    getPower(uint16_t motorID); // 24 (0x71) Motor power acquisition
    resetProgram(uint16_t motorID); // 25 (0x76) System reset command
    brakeRelease(uint16_t motorID); // 26 (0x77) System brake release command
    brakeLock(uint16_t motorID); // 27 (0x78) System brake lock command
    runtime(uint16_t motorID); // 28 (0xB1) System runtime read command
    softwareVersion(uint16_t motorID); // 29 (0xB2)System software version date read command
    setProtectionTime(uint16_t motorID, uint32_t canRecvTime_MS); // 30 (0xB3) Communication interruption protection time setting command
    setBAUDRate(uint16_t motorID, uint8_t baudrate); // 31 (0xB4) Communication baud rate setting command
    getModel(uint16_t motorID); // 32 (0xB5) Motor model reading command
	functionControl(uint16_t motorID, uint8_t index, uint32_t input); // 33 (0x20) 33 Function control command 
	
    // Other Commands
    setCANID(uint16_t motorID, uint8_t CANID, uint8_t readWriteFlag); // (0x79) CANID setting command 
    motionModeControl(uint16_t motorID, uint16_t pdes, uint16_t vdes, uint16_t kp, uint16_t kd, uint16_t tff); // (0x400 + ID) Motion Mode Control Command_CAN
    setRS485ID(uint16_t motorID); // (0x79) RS485-ID setting command

    // Kevin's Functions
	checkReceived(bool print, byte canReply[]);
	uint64_t checkReceived(bool print);
	sendUnidCANMessage(uint16_t motorID, unsigned char cmsg[]); // Send arbitrary CAN message
	serialFeedback();
	
	
	private:
	
};
#endif
