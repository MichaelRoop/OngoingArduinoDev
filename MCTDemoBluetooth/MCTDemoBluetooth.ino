// Name:	MCTDemoBluetooth.ino
// Created:	12/12/2019 3:14:05 PM
// Author:	Michael
//
// Part of MultiCommTerminal demo to receive commands and send responses
// to and from Arduino Bluetooth shield via serial monitor
//
// Written and tested in Visual Studio using Visual Micro
// Tested against Arduino Uno and itea Bluetooth shield with HC-05 module
//
// Board toggled to DATA and jumpers set to IO-4 TX and IO-5 RX
//
//      0    6
//		같�|같  TX
//		같|같�
//		같같같  RX
//
// MUST HAVE DEBUG SERIAL SET TO 9600 Baud
// MUST HAVE BT SERIAL SET TO 38400 Baud

#include <SoftwareSerial.h>

// The jumpers on BT board are set to 4TX and 5RX. 
// They are reversed on serial since RX from BT gets TX to serial
SoftwareSerial btSerial(5, 4); //RX,TX

#ifndef SECTION_DATA

#define IN_BUFF_SIZE 50
#define MSG_COMP_BUFF 50
char inBuff[IN_BUFF_SIZE];
char msgCmpBuff[IN_BUFF_SIZE];
int msgSize = 0;
unsigned char inIndex = 0;

// Command set expected from Demo App
char OPEN_DOOR_CMD_REV[] = "OpenDoor";
char CLOSE_DOOR_CMD_REV[] = "CloseDoor";
char OPEN_DOOR_CMD[] = "OpenDoor";
char CLOSE_DOOR_CMD[] = "CloseDoor";

int  OPEN_CMD_LEN = 8;
int CLOSE_CMD_LEN = 9;

#define BT_BAUD 115200
#define DBG_BAUD 115200

#define DBG_ON 1

#endif // !SECTION_DATA

#ifndef SECTION_ARDUINO_FUNCS

void setup() {
	SetupCommunications(DBG_BAUD, BT_BAUD);
}


void loop() {
	ListenForData();
	delay(1);
}

#endif // !SECTION_ARDUINO_FUNCS


#ifndef SECTION_HELPERS

void SetupCommunications(long dbgBaud, long btBaud) {
#ifdef DBG_ON
	Serial.begin(dbgBaud);
	while (!Serial) {}
	Serial.println("Debug serial active");
#endif // DBG_ON

	btSerial.begin(btBaud);
	while (!btSerial) {}

	#ifdef DBG_ON
	Serial.println("Bluetooth DATA Mode up and running");
	#endif // DBG_ON
}


void ListenForData() {
	int available = btSerial.available();
	if (available > 0) {
		msgSize = 0;

		// Error check to avoid overrun of buffer
		if ((inIndex + available) > IN_BUFF_SIZE) {
			Blink();
			btSerial.write("ERROR-PURGING INPUT\r\n");

			#ifdef DBG_ON
			Serial.write("ERROR-PURGING INPUT\r\n");
			#endif // DBG_ON

			inIndex = 0;
			return;
		}
		#ifdef DBG_ON
		Serial.print("Received bytes:"); Serial.println(available);
		#endif // DBG_ON

		size_t count = btSerial.readBytes(&inBuff[inIndex], available);
		inIndex += count;
		for (int i = 0; i < inIndex; i++) {
			#ifdef DBG_ON
			DBG_DumpChar(inBuff[i]);
			#endif // DBG_ON

			// Make assumption that \n\r comming in so look for \r for end
			if (i > 1) {
				if (inBuff[i - 1] == '\n' && inBuff[i] == '\r') {
					msgSize = i - 1;
					memset(msgCmpBuff, 0, MSG_COMP_BUFF);
					memcpy(msgCmpBuff, inBuff, msgSize);
					memmove(inBuff, &inBuff[i + 1], (inIndex + count) - (msgSize + 2));
					inIndex -= msgSize + 2;
					memset(&inBuff[inIndex], 0, (IN_BUFF_SIZE - inIndex));
					CompareForResponse(msgSize);
				}
			}
		}
		#ifdef DBG_ON
		Serial.println();
		#endif // DBG_ON
	}
}


/// <summary>Compare the incoming message to carry out IO actions</summary>
/// <param name="msgSize">Size of the incoming message</param>
void CompareForResponse(int msgSize) {

#ifdef DBG_ON
	Serial.println();
	Serial.print("Comparing msg in buffer (");
	Serial.print(msgCmpBuff);
	Serial.println(")");
#endif // DBG_ON

	// Compare from start of buffer. Garbage at end of Command
	// and before terminator is ignored (OpenDoorlsdlfkdjdflj)
	if (strncmp(msgCmpBuff, OPEN_DOOR_CMD, OPEN_CMD_LEN) == 0) {
		Blink();
		btSerial.write("OPENING\n\r");
		btSerial.flush();
		OpenGarageDoor();
	}
	else if (strncmp(msgCmpBuff, CLOSE_DOOR_CMD, CLOSE_CMD_LEN) == 0) {
		Blink();
		btSerial.write("CLOSING\n\r");
		btSerial.flush();
		CloseGarageDoor();
	}
	else {
		Blink();
		btSerial.write("NOT_HANDLED\n\r");
		btSerial.flush();

#ifdef DBG_ON
		Serial.println();
		Serial.println("NOT_HANDLED");
#endif // DBG_ON
	}
}


void OpenGarageDoor() {
#ifdef DBG_ON
	Serial.println();
	Serial.println("OPENING Garage Door");
#endif // DBG_ON

	// Do you IO stuff here to open the door
}


void CloseGarageDoor() {
#ifdef DBG_ON
	Serial.println();
	Serial.println("CLOSING Garage Door");
#endif // DBG_ON

	// Do you IO stuff here to close the door
}

// So user can tell device is sending back stuff
void Blink() {
	digitalWrite(LED_BUILTIN, HIGH);
	delay(1);
	digitalWrite(LED_BUILTIN, LOW);
}


void DBG_DumpChar(char c) {
#ifdef DBG_ON
	if (c == '\r') {
		Serial.print("CR");
	}
	else if (c == '\n') {
		Serial.print("LN");
	}
	else {
		Serial.print(c);
	}
#endif // DBG_ON
}


#endif // !SECTION_HELPERS



