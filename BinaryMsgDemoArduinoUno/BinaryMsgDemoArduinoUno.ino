/*
 Name:		BinaryMsgDemoArduinoUno.ino
 Created:	10/11/2021 10:09:24 AM
 Author:	Michael
*/


// Add the define in the project DEFINE property field to enable in ino and lib
//#BINARY_MSG_DEBUG

#include <BinaryMsgProcessor.h>
#include "TempProcessing.h"

#ifndef SECTION_DEFINES

#ifdef __AVR__
	// UNO and others use the software serial.
	#include <SoftwareSerial.h>

	// Must set the Bluetooth baud via an AT command 
	//    On HC-05 AT+UART=115200,1,0
	//    On HC-06 AT+BAUD8
	// Otherwise it is not fast enough. SoftwareSerial does corrupts 
	// data at that speed but I can recover. Better to route Bluetooth 
	// to use a hardware serial port but this does for demo
	#define BT_BAUDRATE 115200
	#define DBG_BAUD 115200

	// The jumpers on Bluetooth board are set to 4TX and 5RX. 
	// They are reversed on serial since RX from BT gets TX to serial
	SoftwareSerial _btSerial(5, 4); //RX,TX
#define btSerial _btSerial 
#else 
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
	#define BT_BAUDRATE 115200
	#define DBG_BAUD 115200
#endif // AVR

#include <BinaryMsgMessages.h>

// Outgoing msg IDs
#define OUT_MSG_ID_ANALOG_0 20

// MSG IDs for incoming message
#define IN_MSG_ID_LED_RED_PIN 10
#define IN_MSG_ID_LED_BLUE_PIN 11
#define IN_MSG_ID_PMW_PIN_X 12
#define IN_MSG_ID_PMW_PIN_Y 13

// Arduino physical pins
// Due reserves 01 for programing port?
#define LED_RED_PIN 2
#define LED_BLUE_PIN 3
#define PMW_PIN_X 9
#define PMW_PIN_Y 10

// Analog debounce limit
#define ANALOG_DEBOUNCE_GAP 5

// We never copy in more than the max current message size of 12
#define IN_BUFF_SIZE 12

#endif // !SECTION_DEFINES

#ifndef SECTION_VARIABLES

int lastA0Value;
int lastPinX;

// Just have the types you need in your application. One can be used for 
// multiple outgoing IOs by changing the ID and Value before send
MsgFloat32 outFloat;
MsgUInt8 outUint8;
//MsgBool outBool;

// Process the temperature
TemperatureProcessor temperatureProcessor;

// In buffer. Currently largest message is 12 bytes. We can just copy to 
// A specific message in a function to avoid reserving that memory
uint8_t buff[IN_BUFF_SIZE];
uint8_t currentPos = 0;
uint8_t currentRemaining = 0;

#endif // !SECTION_VARIABLES

#ifndef SECTION_ARDUINO_FUNCTIONS

void setup() {
#ifdef BINARY_MSG_DEBUG
	Serial.begin(DBG_BAUD);
	while (!Serial) {}
#endif // BINARY_MSG_DEBUG

	btSerial.begin(BT_BAUDRATE); // 
#ifdef __AVR__
	// No wait for hardware serial, only soft serial
	while (!btSerial) {}
#endif // __AVR__

#ifdef BINARY_MSG_DEBUG
	Serial.println("BT Binary...");
#endif // BINARY_MSG_DEBUG

	Initialize();
}

void loop() {
	ListenForData();
	CheckForSendBackData();
}

#endif // !SECTION_ARDUINO_FUNCTIONS

#ifndef SECTION_PRIVATE_HELPERS

// First time initializations
void Initialize() {

	// Digital Pins for demo. Will differ per application
	pinMode(LED_RED_PIN, OUTPUT);
	pinMode(LED_BLUE_PIN, OUTPUT);

	// Digital Pins with PWM for demo. Will differ per application
	pinMode(PMW_PIN_X, OUTPUT);
	pinMode(PMW_PIN_Y, OUTPUT);

	// Analog read values. Out of range forces first value to send
	lastA0Value = 0xFFFFFFFF;
	lastPinX = 0xFFFFFFFF;
	ResetInBuff();

	//-------------------------------------------------------------
	// Register expected id/data type combination for incoming msgs
	// Expecting bool to turn LED on or off
	BinaryMsgProcessor::RegisterInMsgId(IN_MSG_ID_LED_BLUE_PIN, typeBool);
	BinaryMsgProcessor::RegisterInMsgId(IN_MSG_ID_LED_RED_PIN, typeBool);

	// Expecting Uint 8 msg with values 0-255 for PWM on digital pin
	BinaryMsgProcessor::RegisterInMsgId(IN_MSG_ID_PMW_PIN_X, typeUInt8);
	BinaryMsgProcessor::RegisterInMsgId(IN_MSG_ID_PMW_PIN_Y, typeUInt8);

	// Will be raised from parsed incoming message if msg value is of certain type
	// Register all you require for your program
	BinaryMsgProcessor::RegisterInMsgHandler_Bool(&InMsgHandler_Bool);
	BinaryMsgProcessor::RegisterInMsgHandler_UInt8(&InMsgHandler_Uint8);
#ifdef BINARY_MSG_DEBUG
	BinaryMsgProcessor::RegisterErrHandler(&DebugErrHanlder);
#endif // BINARY_MSG_DEBUG
}


void ResetInBuff() {
	memset(buff, 0, IN_BUFF_SIZE);
	currentPos = 0;
	currentRemaining = 0;
}


// Purge all in the app buffer and Bluetooth serial buffer
void PurgeBuffAndBT() {
	int available = btSerial.available();
	while (available > 0) {
		// Iterate through max BUFF size read at a time
		if (available > IN_BUFF_SIZE) {
			available = IN_BUFF_SIZE;
		}
		int count = btSerial.readBytes(buff, available);
		available = btSerial.available();
	}
	ResetInBuff();
}

#endif // !SECTION_PRIVATE_HELPERS

#ifndef SECTION_INCOMING_MSGS

void ListenForData() {
	// Loop through until nothing in buffer to minimize overflow
	int available = btSerial.available();
	while (available > 0) {
		available = btSerial.available();
		if (available > 0) {
			if (currentPos == 0) {
				GetNewMsg(available);
			}
			else {
				GetRemainingMsgFragment(available);
			}
		}
	}
}


// New message arriving. Don't pick up until the entire header is in BT buffer
void GetNewMsg(int available) {
	if (available >= MSG_HEADER_SIZE) {
		//Serial.print("GetNewMsg:"); Serial.println(available);
		currentPos = btSerial.readBytes(buff, MSG_HEADER_SIZE);

#ifdef BINARY_MSG_DEBUG
		//DumpBuffer(buff, currentPos);
#endif // BINARY_MSG_DEBUG

		if (BinaryMsgProcessor::ValidateHeader(buff, currentPos)) {
			currentRemaining = (BinaryMsgProcessor::GetSizeFromHeader(buff) - MSG_HEADER_SIZE);
			available = btSerial.available();
			if (available >= currentRemaining) {
				GetRemainingMsgFragment(available);
			}
		}
		else {
#ifdef BINARY_MSG_DEBUG
			Serial.print("GetNewMsgERR- currentPos:"); Serial.println(currentPos);
			//DumpBuffer(buff, currentPos);
#endif // BINARY_MSG_DEBUG

			PurgeBuffAndBT();
		}
	}
}


// Get enough bytes to make a completed message and process result
void GetRemainingMsgFragment(int available) {
	if (available >= currentRemaining) {
		size_t count = btSerial.readBytes(buff + currentPos, currentRemaining);
		//Serial.print("GetFrag:"); Serial.print(currentRemaining); Serial.print(":"); Serial.println(count);
		currentPos += count;
		bool result = BinaryMsgProcessor::ValidateMessage(buff, currentPos);
#ifdef BINARY_MSG_DEBUG
		Serial.print("GetRemainingInFrag: ");
		//DumpBuffer(buff, currentPos);
#endif // BINARY_MSG_DEBUG
		ResetInBuff();
	}
}

#endif // !SECTION_INCOMING_MSGS

#ifndef SECTION_OUTGOING_MSGS

// May have to put these in a stack where the send can happend when other sends complete
void CheckForSendBackData() {
	// In our demo, a KY-013 temperature sensor is attached to Analog pin A0
	if (ChatterFiltered(analogRead(A0), &lastA0Value, OUT_MSG_ID_ANALOG_0)) {
		SendTemperature(lastA0Value);
	}


	//if (ChatterFiltered(analogRead(PMW_PIN_X), &lastPinX, IN_MSG_ID_PMW_PIN_X)) {
	//	SendUint8Msg(IN_MSG_ID_PMW_PIN_X, lastPinX);
	//}
	//int val = analogRead(PMW_PIN_X);
	//if (val != lastPinX) {
	//	lastPinX = val;
	//	SendUint8Msg(IN_MSG_ID_PMW_PIN_X, val);
	//}

	// TODO - put the PWM 12 here
}


bool ChatterFiltered(int current, int* last, uint8_t pinId) {
	if ((current - ANALOG_DEBOUNCE_GAP) > *last ||
		(current + ANALOG_DEBOUNCE_GAP) < *last) {
		*last = current;
		return true;
	}
	return false;
}

/*
// Use any of the following with the specified messge to send info to Dashboard
void SendBoolMsg(uint8_t id, bool value) {
	outBool.Id = id;
	outBool.Value = value;
	SendMsg(&outBool, outBool.Size);
}

void SendInt8Msg(uint8_t id, int8_t value) {
}

void SendInt16Msg(uint8_t id, int16_t value) {
}

void SendInt32Msg(uint8_t id, int32_t value) {
}

void SendUInt16Msg(uint8_t id, uint16_t value) {
}

void SendUInt32Msg(uint8_t id, uint32_t value) {
}
*/

void SendFloatMsg(uint8_t id, float value) {
	outFloat.Id = id;
	outFloat.Value = value;
	//Serial.println(value);

#ifdef BINARY_MSG_DEBUG
	// DO A TEMP HERE. THE SIZE IS RECORDED AS 2 TOO BIG
	//Serial.print("Out Float32 Msg: ");
	DumpBuffer((uint8_t*)&outFloat, outFloat.Size);
#endif // BINARY_MSG_DEBUG

	SendMsg(&outFloat, outFloat.Size);
}


void SendUint8Msg(uint8_t id, uint8_t value) {
	outUint8.Id = id;
	outUint8.Value = value;
	//Serial.println(value);

#ifdef BINARY_MSG_DEBUG
	Serial.print("Out Uint8 Msg: ");
	DumpBuffer((uint8_t*) &outUint8, outUint8.Size);
#endif // BINARY_MSG_DEBUG

	SendMsg(&outUint8, outUint8.Size);
	delay(10);
}


void SendMsg(void* msg, int size) {
	size_t sent = btSerial.write((uint8_t*)msg, size);
	//Serial.print("Size sent:"); Serial.println(sent);
}


// Convert raw sensor to temperature
void SendTemperature(int sensorValue) {
	temperatureProcessor.ProcessRaw(sensorValue);
	SendFloatMsg(OUT_MSG_ID_ANALOG_0, temperatureProcessor.Celcius());
	//SendFloatMsg(ANALOG_0_ID, tempProcessor.Kelvin());
	//SendFloatMsg(ANALOG_0_ID, tempProcessor.Farenheit());
}

#endif // !SECTION_OUTGOING_MSGS

#ifndef SECTION_CALLBACKS

void InMsgHandler_Bool(uint8_t id, bool val) {
#ifdef BINARY_MSG_DEBUG
	Serial.print("InMsgHandler_Bool(id "); Serial.print(id); Serial.print(", val "); Serial.print(val); Serial.println(")");
#endif // BINARY_MSG_DEBUG

	switch (id) {
	case IN_MSG_ID_LED_RED_PIN:
		digitalWrite(LED_RED_PIN, val ? HIGH : LOW);
		break;
	case IN_MSG_ID_LED_BLUE_PIN:
		digitalWrite(LED_BLUE_PIN, val ? HIGH : LOW);
		break;
	default:
		// TODO - error msg if desired
		break;
	}
}


void InMsgHandler_Uint8(uint8_t id, uint8_t value) {
	//Serial.print("U8-id:"); Serial.print(id); Serial.print(" Val:"); Serial.println(value);
	//Analog writes to PWM pin from 0 - 255 (8 bits), reads from 0 - 1023 (10 bits)
	switch (id) {
	case IN_MSG_ID_PMW_PIN_X:
		// Write to pin
		analogWrite(PMW_PIN_X, value);
		// For demo, bounce back the new value
		SendUint8Msg(IN_MSG_ID_PMW_PIN_X, value);
		break;
	case IN_MSG_ID_PMW_PIN_Y:
		analogWrite(PMW_PIN_Y, value);
		break;
	default:
		// TODO - error msg if desired
		break;
	}
}


#ifdef BINARY_MSG_DEBUG
void DebugErrHanlder(ErrMsg* errMsg) {
	PrintErr(errMsg);
	if (errMsg->Error != err_NoErr) {
		Serial.print("-SOH:"); Serial.println(errMsg->SOH);
		Serial.print("-STX:"); Serial.println(errMsg->STX);
		Serial.print("-Size:"); Serial.println(errMsg->Size);
		Serial.print("-Type:"); PrintDataType(errMsg);
		Serial.print("-Payload Size:"); Serial.println(errMsg->PayloadSize);
		Serial.print("-Required Size:"); Serial.println(errMsg->RequiredSize);
		Serial.print("-ID:"); Serial.println(errMsg->Id);
	}
}


void PrintDataType(ErrMsg* msg) {
	switch (msg->DataType) {
	case typeUndefined:
	case typeBool:
		Serial.println("bool");
		break;
	case typeInt8:
		Serial.println("int8");
		break;
	case typeUInt8:
		Serial.println("uint8");
		break;
	case typeInt16:
		Serial.println("int16");
		break;
	case typeUInt16:
		Serial.println("uint16");
		break;
	case typeInt32:
		Serial.println("int32");
		break;
	case typeUInt32:
		Serial.println("uint32");
		break;
	case typeFloat32:
		Serial.println("float32");
		break;
		//case typeString:
	case typeInvalid:
		Serial.println("invalid type");
		break;
	default:
		Serial.println("unhhandled type");
		break;
		break;
	}
}


void PrintErr(ErrMsg* msg) {
	switch (msg->Error) {
	case err_NoErr:
		Serial.println("\nNo err");
		break;
	case err_InvalidType:
		Serial.println("\nInvalid data type");
		break;
	case err_InvalidHeaderSize:
		Serial.println("\nInvalid header size");
		break;
	case err_StartDelimiters:
		Serial.println("\nErr with start delimiters");
		break;
	case err_InvalidSizeField:
		Serial.println("\nInvalid msg size value");
		break;
	case err_InvalidPayloadSizeField:
		Serial.println("\nBad payload size value");
		break;
	case err_InvalidDataTypeForRegisteredId:
		Serial.println("\nInvalid data type for msg");
		break;
	case err_CallbackNotRegisteredForId:
		Serial.println("\nNo callback for msg");
		break;
	default:
		Serial.println("\nUnhandled");
		break;
	}
}


void DumpBuffer(uint8_t* theBuff, uint8_t size) {
	Serial.print("DumpBuffer - size:"); Serial.println(size);
	for (int i = 0; i < size; i++) {
		if (i > 0) {
			Serial.print(",");
		}
		Serial.print(theBuff[i]); 
	}
	Serial.println();
}

#endif

#endif // !SECTION_CALLBACKS
