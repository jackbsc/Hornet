#include "CurrentSensor.h"

CurrentSensor currentSensor;

uint32_t CurrentSensor::getVcc(void){
	rawReading = ADS1115.readADS(MUX_CHAN_2 | PGA_GAIN_2_3 | MODE_CONTIN | DR_860SPS | CQUE_NONE | OS_CONVERSION_START);
	rawReading = constrain(rawReading, 0, 32767);
	Vcc = (uint32_t)rawReading*stepSize_15bit;
	return Vcc;
}

float CurrentSensor::getVout(void) {
	rawReading = ADS1115.readADS(MUX_CHAN_3 | PGA_GAIN_2_3 | MODE_CONTIN | DR_860SPS | CQUE_NONE | OS_CONVERSION_START);
	rawReading = constrain(rawReading, 0, 32767);
	Vout = ((uint32_t)rawReading*stepSize_15bit)*1.0 / 1000000.0;
	return Vout;
}

void CurrentSensor::begin(boolean readCapacity) {
	if (readCapacity == USE_STORED_CAPACITY)
		capacity = eeprom_read_word((uint16_t*)15);
}

uint16_t CurrentSensor::getCapacity(float* _current) {
	startTime = endTime;
	getVout();
	//getVcc();
	endTime = millis();
	//current = 36.7*((float)Vout / (float)Vcc) - 18.3 + 0.2;
	current = 7.7054*Vout - 3.8841;
	current = constrain(current, 0.0, 50.0);
	*_current = current;
	capacity += ((float)endTime - (float)startTime)*current / 1000;
	currentMillis = millis();
	if (currentMillis - previousMillis > 10000) {	//every 10sec
		previousMillis = currentMillis;
		Serial.println("Recorded");
		eeprom_update_word((uint16_t*)15, (uint16_t)capacity);
		eeprom_busy_wait();
	}
	return (uint16_t)capacity;
}

void CurrentSensor::changeCapacity(uint8_t* newCapacity) {
	uint16_t capacity_temp = 0;
	capacity_temp = newCapacity[0];
	capacity_temp = ((uint16_t)newCapacity[1] << 8) | capacity_temp;
	capacity_temp = constrain(capacity_temp, 0, 36000);
	eeprom_update_word((uint16_t*)15, capacity_temp);
	eeprom_busy_wait();
	capacity = (float)capacity_temp;
}