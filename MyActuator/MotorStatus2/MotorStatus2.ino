#include <MyActuator.h>

MyActuator RMDX8 = MyActuator(0x141);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  RMDX8.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  RMDX8.getMotorStatus2();
  RMDX8.checkReceived();
  delay(1000);
  
}
