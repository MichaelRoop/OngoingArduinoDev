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
// The #ifdef SECTION_... are used to hide code with Visual Micro under Visual studio

#ifndef SECTION_DATA

#define IN_BUFF_SIZE 50
char inBuff[IN_BUFF_SIZE];
int msgSize = 0;
unsigned char inIndex = 0;

// Command set expected from Demo App
char OPEN_DOOR_CMD_REV[] = "OpenDoor";
char CLOSE_DOOR_CMD_REV[] = "CloseDoor";
char OPEN_DOOR_CMD[] = "OpenDoor";
char CLOSE_DOOR_CMD[] = "CloseDoor";

int  OPEN_CMD_LEN = 8;
int CLOSE_CMD_LEN = 9;

#ifdef __AVR__
	// For UNO
	#include <SoftwareSerial.h>
	SoftwareSerial _btSerial(5, 4); //RX,TX
	#define btSerial _btSerial 

	// 115200 is fast for BT module with SoftwareSerial. May get data corruption
	#define BT_BAUD 115200
#else
	// For DUE with hardware port

	// For non AVR like DUE defaulting to hardware serial port 3
	// Matches configuration
	//http://arduino-er.blogspot.com/2015/07/connect-arduino-due-with-hc-06.html
	//
	//Arduino Due + HC - 06 (Bluetooth)-echo bluetooth data
	//
	//Serial(Tx / Rx) communicate to PC via USB
	//Serial3(Tx3 / Rx3) connect to HC - 06
	//HC - 06 Rx - Due Tx3
	//HC - 06 Tx - Due Rx3
	//HC - 06 GND - Due GND
	//HC - 06 VCC - Due 3.3V
	#define btSerial Serial3
	#define BT_BAUD 460800
#endif // __AVR__

#define DBG_BAUD 115200

// Uncomment to enable debug feedback through serial
//#define DBG_ON 1

#endif // !SECTION_DATA

#ifndef SECTION_ARDUINO_FUNCS

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
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
#ifdef __avr__
	while (!btSerial) {}
#endif // __avr__

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
			// Echo portion just received
			if (i >= (inIndex - count)) {
				DBG_DumpChar(inBuff[i]);
			}
			#endif // DBG_ON

			// Make assumption that \n\r comming in so look for \r for end
			if (i > 1) {
				if (inBuff[i - 1] == '\n' && inBuff[i] == '\r') {
					msgSize = i - 1;
					#ifdef DBG_ON
						DBG_DumpMsgFragment(msgSize);
					#endif // DBG_ON
					CompareForResponse();

					// Now move everything over and memset end
					memmove(inBuff, &inBuff[i + 1], (inIndex + count) - (msgSize + 2));
					memset(&inBuff[inIndex], 0, (IN_BUFF_SIZE - inIndex));
					inIndex = 0;
					break;
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
void CompareForResponse() {
	// Compare from start of buffer. Garbage at end of Command
	// and before terminator is ignored (OpenDoorlsdlfkdjdflj)
	if (strncmp(inBuff, OPEN_DOOR_CMD, OPEN_CMD_LEN) == 0) {
		SendResponse("OPENING\n\r");
		OpenGarageDoor();
	}
	else if (strncmp(inBuff, CLOSE_DOOR_CMD, CLOSE_CMD_LEN) == 0) {
		SendResponse("CLOSING\n\r");
		CloseGarageDoor();
	}
	else {
		SendResponse("NOT_HANDLED\n\r");
	}
}


void SendResponse(const char* response) {
	Blink();
	btSerial.write(response);
	btSerial.flush();
#ifdef DBG_ON
	Serial.print("Response:"); Serial.println(response);
#endif // DBG_ON

}


void OpenGarageDoor() {
	// Do you IO stuff here to open the door
}


void CloseGarageDoor() {
	// Do you IO stuff here to close the door
}

// So user can tell device is sending back stuff
void Blink() {
	digitalWrite(LED_BUILTIN, HIGH);
	delay(50);
	digitalWrite(LED_BUILTIN, LOW);
}


void DBG_DumpChar(char c) {
#ifdef DBG_ON
	if (c == '\r') {
		Serial.print("\\r");
	}
	else if (c == '\n') {
		Serial.print("\\n");
	}
	else {
		Serial.print(c);
	}
#endif // DBG_ON
}


void DBG_DumpMsgFragment(int msgSize) {
#ifdef DBG_ON
	Serial.println();
	Serial.print("Comparing msg in buffer (");
	Serial.write(inBuff, msgSize);
	Serial.print(") - Size:"); Serial.println(msgSize);
#endif // DBG_ON
}

#endif // !SECTION_HELPERS



