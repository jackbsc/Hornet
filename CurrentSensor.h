#ifndef _CURRENTSENSOR_H
#define _CURRENTSENSOR_H

#include <Arduino.h>
#include "ADS1115.h"

#define stepSize_15bit 187
#define USE_STORED_CAPACITY true
#define USE_ZERO_CAPACITY  false


class CurrentSensor
{
private:
	int16_t rawReading;
	float Vcc;
	float Vout;
	uint32_t startTime = 0;
	uint32_t endTime = 0;
	float current;
	float capacity = 0;
	uint32_t previousMillis = 0;
	uint32_t currentMillis = 0;
	uint32_t getVcc(void);
	float getVout(void);
public:
	void begin(boolean readCapacity);
	uint16_t getCapacity(float* current);
	void changeCapacity(uint8_t* newCapacity);
};

extern CurrentSensor currentSensor;
extern _16bitADC ADS1115;

#endif /*_CURRENTSENSOR_H*/

