/*
 Name:		HardwareUARTSetBlueTooth.ino
 Created:	10/12/2021 11:56:43 AM
 Author:	Michael

 HC-06 is only receive version and has limited AT command set
 //https://mcuoneclipse.com/2013/06/19/using-the-hc-06-bluetooth-module/
 Command			Response			Notes
 AT					OK
 AT+NAME			OKsetName
 AT+NAMENewName		OKsetName			No space between command and name
 AT+VERSION			hc01.comV2.0
 AT+PIN1234			OKsetPIN			Sets the module PIN to 1234
 AT+BAUD#			OK{newBaud}			# is 1-1200, 2-2400, 3-4800, 4-9600, 5-19200, 6-38400, 7-57600, 
										  8-115200, 9-230400 A-460800, B-921600, C-1382400

Once you set the BAUD it is the save for AT and CMD

Chinese HC-06 blinks continuously when started. It is in AT mode. When connected it goes immediately to CMD mode


*/
//#include <Arduino.h>

//#include <variant.h>

#define KEY_PIN_PULLUP 9
#define HW_SERIAL Serial3

// First connection
#define HW_SERIAL_BAUD 115200
//#define HW_SERIAL_BAUD 38400
//#define HW_SERIAL_BAUD 9600

//#define SW_SERIAL_BAUD 9600
#define SW_SERIAL_BAUD 115200


#define BUFF_SIZE 50

char buff[BUFF_SIZE];

// the setup function runs once when you press reset or power the board
void setup() {
	
	// TO Set to AT command mode you need a pull up on a pin for the BT key pin
	pinMode(KEY_PIN_PULLUP, OUTPUT);
	digitalWrite(KEY_PIN_PULLUP, HIGH);

	HW_SERIAL.begin(HW_SERIAL_BAUD);

	Serial.begin(SW_SERIAL_BAUD);
	while (!Serial) {}


	Serial.println("Enter your AT commands");
}

//int count = 0;

// the loop function runs over and over again until power down or reset
void loop() {
	// Need to empty the complete command response before looking for a new 
	// incomming AT command from terminal.
	// Do NOT include the CR/LN at end of command


	// too slow at 9600
	//int available = HW_SERIAL.available();
	//if (available) {
	//	HW_SERIAL.readBytes(buff, available);
	//	Serial.write(buff, available);
	//	Serial.println("");
	//}

	//available = Serial.available();
	//if (available) {
	//	Serial.readBytes(buff, available);
	//	HW_SERIAL.write(buff, available);
	//}


	//if (HW_SERIAL.available()) {
	//	while (HW_SERIAL.available()) {
	//		Serial.write(HW_SERIAL.read());
	//		delay(3);
	//	}
	//	// Add carriage return
	//	Serial.println("");
	//}

	//if (Serial.available()) {
	//	while (Serial.available()) {
	//		HW_SERIAL.write(Serial.read());
	//	}
	//}


	if (HW_SERIAL.available()) {
		while (HW_SERIAL.available()) {
			Serial.write(HW_SERIAL.read());
			delay(3);
		}
		// Add carriage return
		Serial.println("");
	}

	if (Serial.available()) {
		while (Serial.available()) {
			HW_SERIAL.write(Serial.read());
			delay(3);
		}
	}





}
