#ifndef _DATATYPE_H
#define _DATATYPE_H

#include <Arduino.h>
#include <Servo.h>

typedef Servo Thruster;			//redefinition of servo to thruster

#define CON_VERSION  3			//header connection version

#if CON_VERSION == 1

#define MAIN_LT		 7			//physical pin connection of thrusters
#define MAIN_RT		 6
#define SIDE_LT		 5
#define SIDE_RT		 4

#elif CON_VERSION == 2

#define LED_100		 7
#define LED_80		 6
#define LED_60		 5
#define LED_40		 4

#define MAIN_LT		 5			//physical pin connection of thrusters
#define MAIN_RT		 4
#define SIDE_LT		 39
#define SIDE_RT		 45

#elif CON_VERSION == 3

#define MAIN_LT		44
#define MAIN_RT		45
#define SIDE_LT		46
#define SIDE_RT		48

#define LED_100		7
#define LED_80		6
#define LED_60		5
#define LED_40		4

#define BOARD_LED	47

#endif

#define DEPTH_PIN	 A4			//physical pin connection of depth sensor

#define MAIN_L		 0b11111110 //logical definition of thrusters
#define MAIN_R		 0b11111101
#define SIDE_L		 0b11111011
#define SIDE_R		 0b11110111

#define PULSE_LOW_LIMIT		1100	//pulse width limits definition
#define PULSE_HIGH_LIMIT	1900
#define LOW_SIGNAL_DEAD		1500-30
#define HIGH_SIGNAL_DEAD	1500+30

#define CMD_CAL         0x01
#define CMD_MEAN        0x02
#define CMD_MEDIAN      0x04
#define CMD_CAPACITY	0x08

#define SUCCESS			0b00000000 //response error definition
#define CHECKSUM_ERROR  0b00000001	
#define NO_DEVICE	    0b00000010
#define INVALID_PULSE   0b00000100
#define INVALID_WIDTH	0b00001000
#define DATA_INCOMPLETE 0b00010000
#define TIMEOUT			0b00100000
#define FRAME_ERROR     0b01000000
#define CMD_ERROR       0b10000000

#define NO_DATA		 1
#define NOT_READY    2

#endif /*_DATATYPE_H*/

