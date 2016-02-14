#include "DepthSensor.h"
#include "ADS1115.h"

DepthSensor depthSensor;						   //create depth sensor instance

void DepthSensor::attach(uint8_t pin){
	_pin = pin;
#if ALGORITHM == 0
	_calVolt = eeprom_read_dword((uint32_t*)5);			//read the last saved value
	_calMin = _calVolt;
	_calMax = 4500000 - (500000 - _calVolt);

#elif ALGORITHM == 1
	_calVolt = eeprom_read_float((float*)5);			//read the last saved value
	_calMin = _calVolt;

#endif

}

uint16_t DepthSensor::getRawReading(uint8_t channel){
#if ADC == ONBOARD
	_rawReading = analogRead(_pin);
#elif ADC == ads1115
	channel &= 0x03;
	channel |= 0x04;
	_rawReading = ADS1115.readADS(((uint16_t)channel << 12) | PGA_GAIN_2_3 | MODE_CONTIN | DR_860SPS | CQUE_NONE | OS_CONVERSION_START);
#endif
	_rawReading = constrain(_rawReading, 0, 32767);
	return _rawReading;
}
uint32_t DepthSensor::getVcc(void) {
	getRawReading(2);	//Vcc connected to channel 2
#if ADC == ONBAORD
	_vcc = rawReading*stepSize_10bit;
#else
	_vcc = (uint32_t)_rawReading*stepSize_15bit;
#endif
	return _vcc;
}
uint32_t DepthSensor::getVolts(void){
	getRawReading(0);		//depthsensor input connected to channel 0
#if ADC == ONBOARD
	_volts = _rawReading*stepSize_10bit;			//volts are measured in micro-volts
#elif ADC == ads1115
	_volts = (uint32_t)_rawReading*stepSize_15bit;
#endif
	return _volts;
}

/*get result by mean technique*/
float DepthSensor::calibrateByMean(void) {
	_calVolt = 0;
	for (uint8_t i = 0; i < 10; i++)
		_calVolt += getVolts();
	_calVolt /= 10;
#if ALGORITHM == 0
	_calMin = _calVolt / 10;
	_calMax = 4500000 - (500000 - _calMin);

#elif ALGORITHM == 1
	getVcc();
	float voltsf = (float)_calVolt / 1000000.0;	//convert to float point and actual volts
	float vccf = (float)_vcc / 1000000.0;
	_calMin = (voltsf - 0.1*vccf)*735.19 / vccf;//map to original equation to get Pmin

#endif

	return _calMin;	
}

int32_t DepthSensor::getCMByMean(void){
	
	uint32_t volts = 0;
	for (uint8_t i = 0; i < 10; i++)	   //average out 10 samples
		volts += getVolts();
	volts /= 10;
#if ALGORITHM == 0
	int32_t height = 0;
	//map micro-volt readings to height of water (in micro-meter)
	//maximum volts produce 70555776 micro-meter (rounded to cm)
	height = mapl(volts, _calMin, _calMax, 0, 7055);
	height = constrain(height, 0, 9000);
	return height;
#elif ALGORITHM == 1
	getVcc();								//get supply voltage
	float voltsf = (float)volts / 1000000.0;	//convert to float point and actual volts
	float vccf = (float)_vcc / 1000000.0;
	float pressure = (voltsf - 0.1*vccf)*735.19 / vccf;
	pressure -= _calMin;	//minus the difference of the calibrated volts
	float depth = pressure * 1000 / RHO_G;		//use the calibrated pressure to calculate actual depth
	Serial.println(depth);
	depth = constrain(depth, 0.0, 70.0);
	return (int32_t)(depth * 100.0 + 0.5);		//return depth in cm

#else 
	float voltsf = (float)volts / 1000000.0;	//convert to float point and actual volts
	float depth = 1421.3*voltsf - 650.55; //map to depth by formula
	depth = constrain(depth, 0.0, 70.0);
	return (uint32_t)floor(depth*100.0 + 0.5);

#endif
}

int32_t DepthSensor::getMeterByMean(void) {

	////float height = ((((volts - 0.5) / 0.01)*1723.69) / (997 * 9.807)) * 100;
	//float height = pressure / (9.807*997.048);

	return getCMByMean() / 100;
}
/*end of get result by mean technique*/

/*get result by median technique*/
int16_t compare(const void* p1, const void* p2) {
	if (*(uint32_t*)p1 > *(uint32_t*)p2)
		return 1;
	else if (*(uint32_t*)p1 < *(uint32_t*)p2)
		return -1;
	else
		return 0;
}

float DepthSensor::calibrateByMedian(void) {

	uint32_t volts[9];
	for (uint8_t i = 0; i < 9; i++)
		volts[i] = getVolts();

	qsort(volts, 9, sizeof(uint32_t), compare);
	_calVolt = volts[4];

#if ALGORITHM == 0
	_calMin = _calVolt;
	_calMax = 4500000 - (500000 - _calMin);
#elif ALGORITHM == 1
	getVcc();
	float voltsf = (float)_calVolt / 1000000.0;	//convert to float point and actual volts
	float vccf = (float)_vcc / 1000000.0;
	_calMin = (voltsf - 0.1*vccf)*735.19 / vccf;//map to original equation to get Pmin 

#endif

	return _calMin;
}

int32_t DepthSensor::getCMByMedian(void) {
	uint32_t volts[9];
	uint32_t volts_median;
	for (uint8_t i = 0; i < 9; i++)
		volts[i] = getVolts();

	qsort(volts, 9, sizeof(uint32_t), compare);
	volts_median = volts[4];
#if ALGORITHM == 0
	int32_t height;
	height = mapl(volts_median, _calMin, _calMax, 0, 7055);
	height = constrain(height, 0, 9000);
	if (height == 9000)
		height = 0;
	return height;

#elif ALGORITHM == 1
	getVcc();
	float voltsf = (float)volts_median / 1000000.0;	//convert to float point and actual volts
	float vccf = (float)_vcc / 1000000.0;
	float pressure = (voltsf - 0.1*vccf)*735.19 / vccf;
	pressure -= _calMin;
	float depth = pressure * 1000.0 / RHO_G; //use the calibrated pressure to calculate actual depth

	depth = constrain(depth, 0.0, 70.0);
	
	return (int32_t)(depth * 100.0 + 0.5);		//return depth in cm
#else
	float voltsf = (float)volts_median / 1000000.0;	//convert to float point and actual volts
	float depth = 1421.3*voltsf - 650.55; //map to depth by formula
	depth = constrain(depth, 0.0, 70.0);
	return (uint32_t)floor(depth*100.0 + 0.5);

#endif

}

int32_t DepthSensor::getMeterByMedian(void) {
	return getMeterByMedian() / 100;
}

/*end of get result by median technique*/

int32_t DepthSensor::mapl(int64_t x, int64_t in_min, int64_t in_max, int64_t out_min, int64_t out_max) {

	if ((in_max - in_min) > (out_max - out_min)) 
		return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
	else
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}

float DepthSensor::mapf(float x, float in_min, float in_max, float out_min, float out_max) {

	if ((in_max - in_min) > (out_max - out_min)) 
		return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
	else
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}

