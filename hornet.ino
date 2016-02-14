#include <avr/wdt.h>
#include <Servo.h>
#include <Wire\Wire.h>
#include "DataType.h"
#include "DepthSensor.h"
#include "ADS1115.h"
#include "Console.h"
#include "CurrentSensor.h"

Thruster mainLeft, mainRight, sideLeft, sideRight; //create thruster instance

//depthsensor data log
#define DATALOG 0
//debug on off
#define DEBUG 0

//Forward connection: ESC----->Thruster
//					  Red----->White
//					  Yellow-->Blue
//					  Black--->Green

//global variable definition
boolean calDepth;
uint8_t adcFilter = CMD_MEAN;
uint32_t previousMillis;

void setup() {
#if CON_VERSION!=1
	//setup I/O
	pinMode(LED_100, OUTPUT);
	pinMode(LED_80, OUTPUT);
	pinMode(LED_60, OUTPUT);
	pinMode(LED_40, OUTPUT);
	pinMode(BOARD_LED, OUTPUT);
#endif
	//watch dog timer initialization
	wdt_reset();
	wdt_enable(WDTO_2S);

	//ADS1115 initialization
	ADS1115.begin();

	//variable initilization
	adcFilter = CMD_MEDIAN;
	calDepth = false;
	previousMillis = 0;

	//light up onboard LED
	pinMode(19, OUTPUT);
	digitalWrite(19, HIGH);
	pinMode(47, OUTPUT);
	digitalWrite(47, HIGH);

	//initialize serial port
	Serial.end();
	Serial.begin(115200);

	//depth sensor pin attachment
	depthSensor.attach(DEPTH_PIN);
	//depthSensor.calibrateByMedian();
	//depthSensor.calibrateByMedian();

	//current sensor initialization
	currentSensor.begin(USE_STORED_CAPACITY);

	//thruster pin attachment
	mainLeft.attach(MAIN_LT, PULSE_LOW_LIMIT, PULSE_HIGH_LIMIT);
	mainRight.attach(MAIN_RT, PULSE_LOW_LIMIT, PULSE_HIGH_LIMIT);
	sideLeft.attach(SIDE_LT, PULSE_LOW_LIMIT, PULSE_HIGH_LIMIT);
	sideRight.attach(SIDE_RT, PULSE_LOW_LIMIT, PULSE_HIGH_LIMIT);

	//initialize esc
	mainLeft.writeMicroseconds(1500);
	mainRight.writeMicroseconds(1500);
	sideLeft.writeMicroseconds(1500);
	sideRight.writeMicroseconds(1500);

}

#if DATALOG ==1
void loop() {
	wdt_disable();
	Serial.println("Depth Sensor Data Logging....");
	delay(500);
	Serial.println("3");
	delay(1000);
	Serial.println("2");
	delay(1000);
	Serial.println("1");
	delay(1000);
	Serial.println("Logging Begins....");
	Serial.println("Counting Down....");
	Serial.println(10, DEC);
	for (uint8_t i = 0; i < 20; i++) {
		uint32_t depthData = depthSensor.getVolts();
		eeprom_update_dword((uint32_t*)(i + sizeof(uint32_t)), depthData);
		eeprom_busy_wait();
		delay(500);
		if (i % 2 == 0 && i != 0) {
			Serial.println(10 - (i / 2) - 1, DEC);
		}
	}
	Serial.println("Depth Sensor Data Logging Done....");
	Serial.println("Press Any Key to Go Again....");
	while (!Serial.available());
	while (Serial.available())
		Serial.read();
}

#elif DEBUG == 0

void loop() {
	
	int8_t  readStatus;
	uint8_t error;
	uint8_t bytesRead;
	uint8_t deviceByte;
	uint8_t dataByte[8] = { 0 };
	uint8_t checkSumByte;
	uint8_t startDelimiter;
	uint32_t currentMillis;
	uint8_t cmdByte = 0;
	uint8_t numOfLED[6] = { 4,3,2,1,0,0 };

	wdt_reset();						//clear watchdog timer

	float current;
	uint16_t capacity = currentSensor.getCapacity(&current);	//get the current battery capacity

#if CON_VERSION!=1
	lightupLED(numOfLED[capacity / 7200]);
#endif

	if (calDepth == false) {			//if no calibrate depth sensor command is received
		currentMillis = millis();		//send depth and battery data at approx. 40Hz
		if (currentMillis - previousMillis >1000) {
			console.sendDepthData(adcFilter == CMD_MEAN ? depthSensor.getCMByMean() : depthSensor.getCMByMedian()); //choose which depth data to send according to the filter set
			console.sendCapacity(capacity);
			console.sendCurrent(current);
			//if (current > 20.0) {				//if instantaneous current overshoot
			//	for (uint8_t i = 0; i < 4; i++) {
			//		dataByte[2 * i + 1] = 0x05;
			//		dataByte[2 * i] = 0xDC;
			//	}
			//	console.setThrusterSpeed((uint8_t*)10, (uint8_t*)0xF0, dataByte);//stop thruster
			//}
			previousMillis = currentMillis;
		}
	}
	else {
		depthSensor.calibrateByMedian();
#if ALGORITHM == 0
		uint32_t calvolt = depthSensor.calibrateByMedian();
		eeprom_update_dword((uint32_t*)5, calvolt);		//save the value to eeprom

#elif ALGORITHM == 1
		float calvolt = depthSensor.calibrateByMean();
		eeprom_update_float((float*)5, calvolt);		//save the value to eeprom
#else

#endif
		eeprom_busy_wait();
		calDepth = false;
		console.sendResponse(SUCCESS);		//send success response after calibration complete
	}
	
	readStatus = console.nonBlockingRead(&bytesRead, &deviceByte, dataByte, &checkSumByte, &startDelimiter);	 //read commands from raspberry pi

	if (readStatus != SUCCESS && readStatus != NOT_READY && readStatus != NO_DATA)
		console.sendResponse(readStatus);	//if read unsuccessfull, send back error
	else if(readStatus == SUCCESS) {
		error = console.checkError(&bytesRead, &deviceByte, dataByte, &checkSumByte, &startDelimiter);				//check command integrity
		console.sendResponse(error);	 //send back respective error
		if (error == 0 && startDelimiter == 'T')//if no error and thruster command
			console.setThrusterSpeed(&bytesRead, &deviceByte, dataByte);//set thruster speed
		else if (error == 0 && ((startDelimiter == 'C') || (startDelimiter == 'B'))) {
			if (startDelimiter == 'B') {//if battery command (only 1 for now)
				currentSensor.changeCapacity(dataByte);//change the initial capacity
				console.sendResponse(SUCCESS);
			}
			else {						//analyze depth command
				cmdByte = dataByte[0];

				if ((cmdByte & CMD_CAL) == CMD_CAL)
					calDepth = true;
				if ((cmdByte & CMD_MEAN) == CMD_MEAN) {
					adcFilter = CMD_MEAN;
				}
				if ((cmdByte & CMD_MEDIAN) == CMD_MEDIAN) {
					adcFilter = CMD_MEDIAN;
				}
			}
		}
	}
}

#if CON_VERSION !=1
void lightupLED(uint8_t number) {

	uint32_t currentMillis;
	static uint32_t previousMillis = 0;
	static uint8_t status = LOW;

	switch (number) {
	case 1:
		digitalWrite(LED_100, LOW);
		digitalWrite(LED_80, LOW);
		digitalWrite(LED_60, LOW);
		digitalWrite(LED_40, HIGH);
		break;
	case 2:
		digitalWrite(LED_100, LOW);
		digitalWrite(LED_80, LOW);
		digitalWrite(LED_60, HIGH);
		digitalWrite(LED_40, HIGH);
		break;
	case 3:
		digitalWrite(LED_100, LOW);
		digitalWrite(LED_80, HIGH);
		digitalWrite(LED_60, HIGH);
		digitalWrite(LED_40, HIGH);
		break;
	case 4:
		digitalWrite(LED_100, HIGH);
		digitalWrite(LED_80, HIGH);
		digitalWrite(LED_60, HIGH);
		digitalWrite(LED_40, HIGH);
		break;
	default:
		currentMillis = millis();
		if (currentMillis - previousMillis > 500) {
			previousMillis = currentMillis;
			status = !status;
			digitalWrite(LED_100, status);
			digitalWrite(LED_80, status);
			digitalWrite(LED_60, status);
			digitalWrite(LED_40, status);
		}
	}
}
#endif

#elif DEBUG ==1



#endif