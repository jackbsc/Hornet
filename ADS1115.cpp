#include "ADS1115.h"

_16bitADC ADS1115;

void _16bitADC::begin(void) {
	Wire.begin();
	Wire.setClock(400000L); //set to 400KHz speed
}

void _16bitADC::writeReg(uint8_t addr, uint8_t reg, uint16_t value) {
	Wire.beginTransmission(addr);
	Wire.write(reg);					//points to config register
	Wire.write((uint8_t)(value >> 8));
	Wire.write((uint8_t)(value & 0xFF));
	Wire.endTransmission();
}

int16_t _16bitADC::readReg(uint8_t addr, uint8_t reg) {

	uint8_t highByte;
	uint8_t lowByte;

	Wire.beginTransmission(addr);
	Wire.write(reg);					//points to convert register
	Wire.endTransmission(false);		//sends a restart message

	Wire.requestFrom(addr, (uint8_t)2);
	highByte = Wire.read();
	lowByte = Wire.read();
	Wire.endTransmission();

	return ((int16_t)((highByte << 8) | lowByte));
}

void _16bitADC::BusyConversion(uint8_t addr) {

	uint8_t highByte;
	uint8_t lowByte;
	uint16_t busy;
	boolean sameChannel = false;

	Wire.beginTransmission(addr);
	Wire.write(REG_POINTER_CONFIG);
	Wire.endTransmission(false);

	Wire.requestFrom(addr, (uint8_t)2);
	/*check for conversion ready*/
	do {
		highByte = Wire.read();
		lowByte = Wire.read();
		busy = ((uint16_t)((highByte << 8) | lowByte));
		if ((busy & 0x8000) == 0) {
			Wire.endTransmission(false);
		}
		else {
			Wire.endTransmission();
		}
	} while ((busy & 0x8000) == 0);
	///*end of check for conversion ready*/

}

int16_t _16bitADC::readADS(uint16_t configVal) {

	uint8_t currentChan = (configVal & 0x7000) >> 12;

	writeReg(ADDRESS, REG_POINTER_CONFIG, configVal);
	if (previousChan != currentChan) {
		delayMicroseconds(800);
		previousChan = currentChan;
	}
	BusyConversion(ADDRESS);
	int16_t result = readReg(ADDRESS, REG_POINTER_CONVERT);

	return result;

}