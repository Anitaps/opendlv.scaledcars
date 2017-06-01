#ifndef SerialControl_h
#define SerialControl_h

#include "Arduino.h"


class SerialControl{
  public:
	SerialControl(String in);
    String getValue(char separator, int stIndex);
	int getSpeed();
	int getAngle();
  private:
	String _input;
    int _maxIndex; 
};

#endif
