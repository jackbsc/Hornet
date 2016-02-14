#ifndef _DEPTH_H
#define _DEPTH_H

#include <Arduino.h>
#include <math.h>
#include "ADS1115.h"

#define ALGORITHM	0

#define stepSize_10bit 4883
#define stepSize_15bit 187
#define RHO_G		   9781.04

#define ONBOARD  0
#define ads1115	 1
#define ADC	ads1115
extern _16bitADC ADS1115;

class DepthSensor{
private:
	uint8_t _pin;
	uint16_t _rawReading;
	uint32_t _vcc;
	uint32_t _volts;
	int32_t _calVolt;
#if ALGORITHM == 0
	int32_t _calMin;
	int32_t _calMax;
#else ALGORITHM == 1
	float _calMin;
	float _calDiff;
#endif
	int32_t mapl(int64_t, int64_t, int64_t, int64_t, int64_t);
	float mapf(float, float, float, float, float);
public:
	void attach(uint8_t pin);
	float calibrateByMean(void);
	float calibrateByMedian(void);
    uint16_t getRawReading(uint8_t channel);
	uint32_t getVcc(void);
    uint32_t getVolts(void);

	/*get result by mean technique*/
	int32_t getCMByMean(void);
	int32_t getMeterByMean(void);

	/*get result by median technique*/
	int32_t getCMByMedian(void);
	int32_t getMeterByMedian(void);

};

extern DepthSensor depthSensor;						   //create depth sensor instance

#endif /*_DEPTH_H*/
