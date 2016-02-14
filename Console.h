#ifndef _CONSOLE_H
#define _CONSOLE_H

#include <Arduino.h>
#include "DataType.h"

class Console
{
private:
	boolean startReceive = false;
	uint8_t buf[12] = { 0 };
	uint8_t bytesToRead = 0;
	uint8_t totalBytesRead = 0;
	boolean continueToRead = false;
	uint8_t index = 0;
	uint8_t delimiter = 0;
	uint32_t previousMillis = 0;
	uint32_t currentMillis = 0;
	uint32_t timeout = 500;			//timeout period set to 500ms
	
	uint8_t formattedRead(uint8_t maxBytes, uint8_t minBytes);
	void houseKeeping(boolean* continueToRead, boolean* startReceive, uint8_t* index, uint8_t* bytesToRead, uint8_t* totalBytesRead, uint8_t* delimiter);

public:
	uint8_t blockingRead(uint8_t* bytesRead, uint8_t* deviceByte, uint8_t* dataByte, uint8_t* checkSumByte, uint8_t* startDelimiter);
	uint8_t nonBlockingRead(uint8_t* bytesRead, uint8_t* deviceByte, uint8_t* dataByte, uint8_t* checkSumByte, uint8_t* startDelimiter);
	uint8_t checkError(uint8_t* bytesRead, uint8_t* deviceByte, uint8_t* dataByte, uint8_t* checkSumByte, uint8_t* startDelimiter);
	void sendResponse(uint8_t errorByte);
	void sendDepthData(int32_t depthData);
	void sendCapacity(uint16_t capacity);
	void sendCurrent(float current);
	void setThrusterSpeed(uint8_t* bytesRead, uint8_t* deviceByte, uint8_t* dataByte);
};

extern Console console;
extern Thruster mainLeft, mainRight, sideLeft, sideRight;

#endif /*_CONSOLE_H*/

