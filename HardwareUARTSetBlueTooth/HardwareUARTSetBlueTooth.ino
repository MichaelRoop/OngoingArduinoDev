/*
 Name:		HardwareUARTSetBlueTooth.ino
 Created:	10/12/2021 11:56:43 AM
 Author:	Michael

 HC-06 is only receive version and has limited AT command set. 
 //https://mcuoneclipse.com/2013/06/19/using-the-hc-06-bluetooth-module/
 //http://wiki.sunfounder.cc/images/7/7b/HC-06_AT_Commands_Reference.pdf
 Command			Response			Notes
 AT					OK
 AT+NAME			OKsetName
 AT+NAMENewName		OKsetName			No space between command and name
 AT+VERSION			hc01.comV2.0
 AT+PIN1234			OKsetPIN			Sets the module PIN to 1234
 AT+BAUD#			OK{newBaud}			# is 1-1200, 2-2400, 3-4800, 4-9600, 5-19200, 6-38400, 7-57600, 
										  8-115200, 9-230400 A-460800, B-921600, C-1382400

HC-05 has full AT set which can be found online
HC-05 boards MUST have CR/LF added on serial monitor output


For oddball ZS non key pin HC-06 board										  
	Default on first start is 9600 baud. On power up it blinks continously indicating it
	is in AT command mode. Once you connect to it via an App it automaticaly puts the red
	light on continuous and sets itself to DATA mode.
	
	Once you set the BAUD it is the save for AT command mode AND DATA mode. You MUST remember 
	this number to send AT commands or Data
boards

For HC-06 key pin boards. 
    Just pull the Key pin HIGH before powering the HC-06 board

*/


#ifdef HC06_WITH_KEY_PIN
	// Using Digital Pin 9 here. Change as wanted
	#define KEY_PIN_PULLUP 9

#endif // HC06_WITH_KEY_PIN

#define USING_HW_SERIAL
#ifdef USING_HW_SERIAL
	// For boards like DUE with hardware serial. Using 3 here. Change as wanted
	#define BLUETOOTH_SERIAL_PORT Serial3
	// For non key pin board like ZS-040 from www.hc02.com this will be the last
	// baud set THAT YOU MUST REMEMBER. 
	#define BT_HC06_SERIAL_BAUD 460800
	// For feeding commands to the board. Set as desired
	// NOTE: For the ZS HC-06 you must NOT add any delimiters (CR/LF) at end of AT commands
	// on the serial output. It assumes a wait of 30ms means full command sent
	#define	SW_SERIAL_BAUD 115200
#else
	#include <SoftwareSerial.h>
	SoftwareSerial softSerial(5, 4); //RX,TX
	#define BLUETOOTH_SERIAL_PORT softSerial
	
	// HC-06 boards with KEY Pins have a set 9600 baud when in AT command mode
	// 
	// For HC-05 shield for UNO requires 38400 to BT in AT mode. Some boards have physical switch
	#define BT_HC06_SERIAL_BAUD 38400
	#define	SW_SERIAL_BAUD 38400
#endif // USING_HW_SERIAL


// First connection


#define BUFF_SIZE 50

char buff[BUFF_SIZE];

// the setup function runs once when you press reset or power the board
void setup() {

#ifdef HC06_WITH_KEY_PIN
	// For HC-06 boards that have a KEY Pin, you need a pull it up 
	// HIGH on a pin. Connect the pin to the 3.3v Pin BEFORE you connect
	// the 3.3v power to the HC-06 board. This will put those boards into
	// AT mode so you can send commands. These boards have a fixed 9600
	// BAUD
	pinMode(KEY_PIN_PULLUP, OUTPUT);
	digitalWrite(KEY_PIN_PULLUP, HIGH);
#endif // HC06_WITH_KEY_PIN

	BLUETOOTH_SERIAL_PORT.begin(BT_HC06_SERIAL_BAUD);
#ifndef USING_HW_SERIAL
	// Only have to wait on software serial port
	while (!BLUETOOTH_SERIAL_PORT) {}
#endif // !USING_HW_SERIAL

	Serial.begin(SW_SERIAL_BAUD);
	while (!Serial) {}

	Serial.println("\nEnter your AT commands");
}


// the loop function runs over and over again until power down or reset
void loop() {

	// Need to empty the complete command response before looking for a new 
	// incomming AT command from terminal.
	// Do NOT include the CR/LN at end of command for ZS HC-06 boards. Maybe other HC-06 also
	if (BLUETOOTH_SERIAL_PORT.available()) {
		while (BLUETOOTH_SERIAL_PORT.available()) {
			Serial.write(BLUETOOTH_SERIAL_PORT.read());
			delay(3);
		}
		// Add carriage return to feedback
		Serial.println("");
	}

	if (Serial.available()) {
		while (Serial.available()) {
			BLUETOOTH_SERIAL_PORT.write(Serial.read());
			delay(3);
		}
	}





}
