#include "Console.h"
typedef Thruster Servo;							   //redefinition of servo to thruster

Console console;								   //create console instance

void Console::houseKeeping(boolean* continueToRead, boolean* startReceive, uint8_t* index, uint8_t* bytesToRead, uint8_t* totalBytesRead, uint8_t* delimiter) {
	*continueToRead = false;
	*startReceive = false;
	*index = 0;
	*bytesToRead = 0;
	*totalBytesRead = 0;
	*delimiter = 0;
}

uint8_t Console::blockingRead(uint8_t* bytesRead, uint8_t* deviceByte, uint8_t* dataByte, uint8_t* checkSumByte, uint8_t* startDelimiter) {

	static boolean startReceive = false;
	uint8_t buf[11];

	if (startReceive == true) {
		startReceive = false;
		if (*startDelimiter == 'T') {						//it is a thruster command
			*bytesRead = Serial.readBytesUntil(0xFF, buf, 11);
			if (*bytesRead == 0xFF)
				return TIMEOUT;							//return timeout
			uint8_t j = Serial.readBytesUntil('E', buf + (*bytesRead), 11 - (*bytesRead));
			if (j != 0xFF) {								//timeout has not occured
				*bytesRead += j;
				if (*bytesRead > 10 || *bytesRead < 4)	//data length exceed maximum range
					return FRAME_ERROR;					//return framing error
				*deviceByte = buf[0];
				for (uint8_t i = 0; i < (*bytesRead) - 1; i++)
					dataByte[i] = buf[i + 1];
				*checkSumByte = buf[(*bytesRead) - 1];
				return SUCCESS;							//return success
			}
			else
				return TIMEOUT;						    //return timeout
		}
		else if (*startDelimiter == 'D') {				//it is a depth sensor command
			*bytesRead = Serial.readBytesUntil('E', buf, 3);
			if (*bytesRead == 0xFF)
				return TIMEOUT;							//return timeout
			if (*bytesRead != 2)
				return CMD_ERROR;						//return command error
			else {
				dataByte[0] = buf[0];
				dataByte[1] = buf[1];
				return SUCCESS;
			}
		}
	}
	if (Serial.available()) {
		*startDelimiter = Serial.read();
		if (*startDelimiter == 'T' || *startDelimiter == 'D') {
			startReceive = true;
			return NOT_READY;						//return not yet done
		}
	}
	return NO_DATA;										//return no data

}

uint8_t Console::formattedRead(uint8_t maxBytes, uint8_t minBytes) {

	bytesToRead = Serial.available();
	if (bytesToRead == 0) {
		return NOT_READY;
	}
	if (bytesToRead + totalBytesRead > maxBytes) {		//get the remaining bytes to read within packet length
		bytesToRead = maxBytes - totalBytesRead;
	}
	uint8_t offsetBytesToRead = bytesToRead + index; //offset the total number of bytes to read according to index location

	for (; index < offsetBytesToRead; index++) {
		buf[index] = Serial.read();
		if (buf[index] == 0xFF) {				//first detect end byte 1
			if (Serial.available()) {			//if next byte is available
				buf[++index] = Serial.read();
				if (buf[index] == 'E') {		//it should be 'E'
					if (index - 1 < minBytes) {		//total bytes read is index-1
						return FRAME_ERROR;
					}
					return SUCCESS;
				}
				else if (buf[index] != 0xFF) {		//or if not equal 0xFF, this is an invalid packet
					return FRAME_ERROR;
				}
				else {									//end byte 2 has not arrived
					totalBytesRead = bytesToRead;		//stored no. of bytes read
					continueToRead = true;				//set to continue reading
					index++;
					return NOT_READY;
				}
			}
			else {									//end byte 2 has not arrived
				totalBytesRead = bytesToRead;		//stored no. of bytes read
				continueToRead = true;				//set to continue reading
				index++;
				return NOT_READY;
			}
		}
		if (continueToRead == true) {			   //end byte 1 has detected
			if (buf[index] == 'E') {			   //last bytes should be 'E'
				if (index - 1 < minBytes) {
					return FRAME_ERROR;
				}
				return SUCCESS;
			}
			else if (buf[index] != 0xFF) {		//or if not equal 0xFF, this is an invalid packet
				return FRAME_ERROR;
			}
		}
	}
	if (offsetBytesToRead > maxBytes) {		  //packet exceed maximum length
		return FRAME_ERROR;
	}

	return NOT_READY;
}

uint8_t Console::nonBlockingRead(uint8_t* bytesRead, uint8_t* deviceByte, uint8_t* dataByte, uint8_t* checkSumByte, uint8_t* startDelimiter) {

	if (startReceive == true) {
		currentMillis = millis();
		if (currentMillis - previousMillis > timeout) {
			houseKeeping(&continueToRead, &startReceive, &index, &bytesToRead, &totalBytesRead, &delimiter);
			return TIMEOUT;
		}
		/*****Thruster Command Processing*****/
		if (delimiter == 'T') {
			uint8_t readStatus = formattedRead(10, 4);
			if (readStatus == SUCCESS) {//read packet and return the data to main
				*bytesRead = index - 1;
				*deviceByte = buf[0];
				*checkSumByte = buf[index - 2];
				*startDelimiter = delimiter;
				for (uint8_t i = 0; i < index - 3; i++)
					dataByte[i] = buf[i + 1];
			}
			if (readStatus != NOT_READY)
				houseKeeping(&continueToRead, &startReceive, &index, &bytesToRead, &totalBytesRead, &delimiter);
			return readStatus;
		}
		/*****End of Thruster Command Processing*****/

		/*****Depth Sensor and Current Sensor Command Processing*****/
		else if (delimiter == 'C') {
			bytesToRead = Serial.available();
			if (bytesToRead == 0)
				return NOT_READY;
			uint8_t offsetBytesToRead = bytesToRead + index; //offset the total number of bytes
			for (; index < offsetBytesToRead; index++) {
				buf[index] = Serial.read();
				if (index == 2) {	//if enough length has read, check for end byte
					if (buf[index] != 'E') {
						houseKeeping(&continueToRead, &startReceive, &index, &bytesToRead, &totalBytesRead, &delimiter);
						return CMD_ERROR;
					}
					dataByte[0] = buf[0];
					dataByte[1] = buf[1];
					*startDelimiter = delimiter;
					houseKeeping(&continueToRead, &startReceive, &index, &bytesToRead, &totalBytesRead, &delimiter);
					return SUCCESS;
				}
				if (buf[index] == 'E') {			//if end byte detected
					if (index != 2) {
						houseKeeping(&continueToRead, &startReceive, &index, &bytesToRead, &totalBytesRead, &delimiter);
						return CMD_ERROR;
					}
					dataByte[0] = buf[0];
					dataByte[1] = buf[1];
					*startDelimiter = delimiter;
					houseKeeping(&continueToRead, &startReceive, &index, &bytesToRead, &totalBytesRead, &delimiter);
					return SUCCESS;
				}
			}
			return NOT_READY;
		}
		/*****End of Depth Sensor Command Processing*****/

		/*****Battery Command Processing****/
		else if (delimiter == 'B') {
			if (Serial.available()) {
				buf[index++] = Serial.read();
				if (index == 4)
					if (buf[index - 1] != 'E') {
						startReceive = false;
						index = 0;
						delimiter = 0;
						return FRAME_ERROR;
					}
					else {
						startReceive = false;
						index = 0;
						dataByte[0] = buf[0];
						dataByte[1] = buf[1];
						*checkSumByte = buf[2];
						*startDelimiter = delimiter;
						return SUCCESS;
					}
			}
			else
				return NOT_READY;
		}
		/*****End of Battery Command Processing*****/
	}

	if (Serial.available() && startReceive == false) {
		delimiter = Serial.read();
		if (delimiter == 'T' || delimiter == 'C') {
			startReceive = true;
			previousMillis = millis();				//record time to check timeout
			return NOT_READY;						//return not yet done
		}
	}
	return NO_DATA;										//return no data

}

uint8_t Console::checkError(uint8_t* bytesRead, uint8_t* deviceByte, uint8_t* dataByte, uint8_t* checkSumByte, uint8_t* startDelimiter) {

	uint8_t datalen = (*bytesRead) - 2;
	uint8_t result = 0;
	uint8_t deviceNum = 0;
	if (*startDelimiter == 'T') {						//it is a thruster command
		for (uint8_t i = 0; i < (*bytesRead); i++) {
			if (i == 0)
				result = (*deviceByte);
			else if (i == 1)
				result ^= (*checkSumByte);
			else
				result ^= dataByte[i - 2];
		}

		if (result != 0) {							  //checksum error
			result = 0;
			result ^= CHECKSUM_ERROR;
		}

		if (*deviceByte < 0xF0 || *deviceByte > 0xFE) //device is out of range
			result ^= NO_DEVICE;

		if (datalen % 2 != 0)
			result ^= DATA_INCOMPLETE;				 //data fragmented
		else {
			for (uint8_t mask = 0b11111110; mask > 0b11101111; mask = (mask << 1) | 1) {
				if ((mask | (*deviceByte)) == mask)
					deviceNum++;
			}
			if (datalen / 2 == deviceNum) {
				for (uint8_t i = 0; i < datalen; i += 2) { //pulse width out of range
					if ((((uint16_t)dataByte[i + 1] << 8) + dataByte[i]) > PULSE_HIGH_LIMIT ||
						(((uint16_t)dataByte[i + 1] << 8) + dataByte[i]) < PULSE_LOW_LIMIT) {
						result ^= INVALID_PULSE;
						break;
					}
				}
			}
			else										//data width incomplete
				result ^= INVALID_WIDTH;
		}
	}
	else if (*startDelimiter == 'C') {					//it is a command byte
		if (dataByte[0] != dataByte[1] || dataByte[0]>0x0F|| dataByte[0] == 0)
			result ^= CMD_ERROR;
		else if ((dataByte[0] & CMD_CAPACITY) == CMD_CAPACITY) {
			delimiter = 'B';
			startReceive = true;
			index = 0;
		}
	}
	else if (*startDelimiter == 'B') {					
		if ((dataByte[0] ^ dataByte[1] ^ *checkSumByte) != 0)
			result ^= CMD_ERROR;
	}
	return result;

}

void Console::sendResponse(uint8_t errorByte) {

	uint8_t buf[5];
	
	buf[0] = 'R';
	buf[4] = '\n';

	for (int8_t i = 3; i > 0; errorByte /= 10, i--)
		buf[i] = errorByte % 10 + '0';

	Serial.write(buf, 5);
}

void Console::sendDepthData(int32_t depthData) {

	uint8_t buf[9];

	buf[0] = 'D';
	for (int8_t i = 3; i >= 0; i--) {
		buf[i + 1] = (depthData % 10) + '0';
		depthData /= 10;
	}
	buf[5] = '\n';

	Serial.write(buf, 6);

}

void Console::sendCapacity(uint16_t capacity) {

	uint8_t buf[8];
	buf[0] = 'B';
	for (int8_t i = 4; i >= 0; i--) {
		buf[i + 1] = (capacity % 10) + '0';
		capacity /= 10;
	}
	buf[6] = '\n';

	Serial.write(buf, 7);
}

void Console::sendCurrent(float current) {
	Serial.write('C');
	if (current < 10) {
		Serial.write('0'); //pad with zero
		Serial.write(((uint8_t)current) + '0');
	}
	else {
		Serial.write(((uint8_t)(current / 10)) + '0');
		Serial.write((((uint8_t)current) % 10) + '0');
	}
	Serial.write('.');
	Serial.write(((uint8_t)(current * 10) % 10) + '0');
	Serial.write(((uint8_t)(current * 100) % 10) + '0');
	Serial.write('\n');
	
}

void Console::setThrusterSpeed(uint8_t* bytesRead, uint8_t* deviceByte, uint8_t* dataByte) {

	uint8_t datalen = (*bytesRead) - 2;
	uint8_t index = 0;
	uint16_t pulseWidth[4];
	uint16_t pulse;
	for (uint8_t i = 0; i < datalen; i += 2) {
		pulse = (uint16_t)((dataByte[i + 1] << 8) + dataByte[i]);
		if (pulse >= LOW_SIGNAL_DEAD && pulse <= HIGH_SIGNAL_DEAD)
			pulseWidth[i / 2] = 1500;
		else
			pulseWidth[i / 2] = pulse;
	}

	if ((*deviceByte | MAIN_L) == MAIN_L) {
		mainLeft.writeMicroseconds(pulseWidth[index]);
		index++;
	}
	if ((*deviceByte | MAIN_R) == MAIN_R) {
		mainRight.writeMicroseconds(pulseWidth[index]);
		index++;
	}
	if ((*deviceByte | SIDE_L) == SIDE_L) {
		sideLeft.writeMicroseconds(pulseWidth[index]);
		index++;
	}
	if ((*deviceByte | SIDE_R) == SIDE_R) {
		sideRight.writeMicroseconds(pulseWidth[index]);
	}
}
