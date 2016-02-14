#ifndef _THRUSTER_H
#define _THRUSTER_H

#include <Arduino.h>
#include <Servo.h>

class Thruster:public Servo {
public:
	void setSpeed(uint16_t setSpeed);
};

extern Thruster mainLeft, mainRight, sideLeft, sideRight; //create thruster instance

#endif /*_THRUSTER_H*/

