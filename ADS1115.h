#ifndef _ADS1115_H
#define _ADS1115_H

#include <Arduino.h>
#include "Wire/Wire.h"

#define ADDRESS					0x48 //addr pin connected to GND
#define REG_POINTER_CONVERT		0x00
#define REG_POINTER_CONFIG      0x01

#define OS_CONVERSION_START		0x8000

#define MUX_CHAN_0					0x4000
#define MUX_CHAN_1					0x5000
#define MUX_CHAN_2					0x6000
#define MUX_CHAN_3					0x7000

#define PGA_GAIN_2_3				0x0000
#define PGA_GAIN_1					0x0200
#define PGA_GAIN_2					0x0400
#define PGA_GAIN_4					0x0600
#define PGA_GAIN_8					0x0800
#define PGA_GAIN_16					0x0A00

#define MODE_SINGLE					0x0000
#define MODE_CONTIN					0x0100

#define DR_8SPS					0x0000
#define DR_16SPS				0x0020
#define DR_32SPS				0x0040
#define DR_64SPS				0x0060
#define DR_128SPS				0x0080
#define DR_250SPS				0x00A0
#define DR_475SPS				0x00C0
#define DR_860SPS				0x00E0

#define CQUE_NONE			0x0003


class _16bitADC
{
private:
	uint8_t _pointerVal;
	uint16_t _configVal;
	uint8_t previousChan = 1;
public:
	void begin(void);
	void writeReg(uint8_t addr, uint8_t reg, uint16_t value);
	void BusyConversion(uint8_t addr);
	int16_t readReg(uint8_t addr, uint8_t reg);
	int16_t readADS(uint16_t configVal);	//read adc conversion result
};

extern _16bitADC ADS1115;

#endif /*_ADS1115_H*/

