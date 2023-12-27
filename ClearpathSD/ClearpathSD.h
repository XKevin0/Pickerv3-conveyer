#include "Arduino.h"

class ClearpathSD{
  public:
    ClearpathSD(int HL, int inpA, int inpB, int en, int pro);
    engage();
	disengage();
    int setAbsPos(long int pos);
    setSpeed(int rpm);
    setAccel(int rpms);
	pulseGen(int setICR1);
	stop();
	long int home();
	setzpos(int zposTemp);
    
  private:
	
    int HLFB;
    int inputB;
    int inputA;
    int enable;
	int prox;
	
    long int zpos;
    int ppr;
	int zaccel;
    double zrpm;
};
